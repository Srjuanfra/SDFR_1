#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/bool.hpp>

#include <cstdint>
#include <cmath>
#include <string>
#include <algorithm>

// ---------------------------------------------------------------------------
// Inline BGR8 -> HSV conversion (no OpenCV, Xenomai-safe)
//
// Algorithm: standard HSV conversion from Foley & Van Dam (1982)
//   H in [0, 360), S in [0, 1], V in [0, 1]
// ---------------------------------------------------------------------------
struct HSV { float h, s, v; };

static inline HSV bgr_to_hsv(uint8_t b, uint8_t g, uint8_t r)
{
  const float rf = r / 255.0f;
  const float gf = g / 255.0f;
  const float bf = b / 255.0f;

  const float cmax = std::max({rf, gf, bf});
  const float cmin = std::min({rf, gf, bf});
  const float diff = cmax - cmin;

  HSV out{0.0f, 0.0f, cmax};

  if (cmax < 1e-6f) return out;          // black pixel, S = H = 0

  out.s = diff / cmax;

  if (diff < 1e-6f) return out;          // grey pixel, H = 0

  if (cmax == rf)
    out.h = 60.0f * std::fmod((gf - bf) / diff, 6.0f);
  else if (cmax == gf)
    out.h = 60.0f * ((bf - rf) / diff + 2.0f);
  else
    out.h = 60.0f * ((rf - gf) / diff + 4.0f);

  if (out.h < 0.0f) out.h += 360.0f;

  return out;
}

// ---------------------------------------------------------------------------
// ObjectPositionNode
//
// Detects a coloured object in a bgr8 image by:
//   1. Converting each pixel to HSV.
//   2. Applying per-channel thresholds to produce a binary mask.
//   3. Computing the centre of gravity (CoG) of the white pixels.
//
// Published topics:
//   /object/position  (geometry_msgs/Point)  — CoG in pixel coordinates
//   /object/found     (std_msgs/Bool)         — true if object detected
//
// Parameters (all settable at launch and at runtime via ros2 param set):
//   image_topic   (string,  default "/image")
//   hue_min       (double,  default  35.0)   — green ball ~40-80°
//   hue_max       (double,  default  85.0)
//   sat_min       (double,  default   0.4)   — reject grey/white
//   sat_max       (double,  default   1.0)
//   val_min       (double,  default   0.2)   — reject very dark pixels
//   val_max       (double,  default   1.0)
//   min_pixels    (int,     default  20)     — CoG not published below this
// ---------------------------------------------------------------------------
class ObjectPositionNode : public rclcpp::Node
{
public:
  ObjectPositionNode() : Node("object_position")
  {
    // ---- declare parameters ------------------------------------------------
    declare_parameter<std::string>("image_topic", "/image");
    declare_parameter<double>("hue_min",   35.0);
    declare_parameter<double>("hue_max",   85.0);
    declare_parameter<double>("sat_min",    0.4);
    declare_parameter<double>("sat_max",    1.0);
    declare_parameter<double>("val_min",    0.2);
    declare_parameter<double>("val_max",    1.0);
    declare_parameter<int>("min_pixels",   20);

    load_parameters();

    // ---- publishers --------------------------------------------------------
    pos_pub_   = create_publisher<geometry_msgs::msg::Point>("/object/position", 10);
    found_pub_ = create_publisher<std_msgs::msg::Bool>("/object/found", 10);

    // ---- subscriber --------------------------------------------------------
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&ObjectPositionNode::imageCallback, this, std::placeholders::_1));

    // ---- runtime parameter updates ----------------------------------------
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &params) {
        for (const auto &p : params) {
          if      (p.get_name() == "hue_min")    hue_min_  = p.as_double();
          else if (p.get_name() == "hue_max")    hue_max_  = p.as_double();
          else if (p.get_name() == "sat_min")    sat_min_  = p.as_double();
          else if (p.get_name() == "sat_max")    sat_max_  = p.as_double();
          else if (p.get_name() == "val_min")    val_min_  = p.as_double();
          else if (p.get_name() == "val_max")    val_max_  = p.as_double();
          else if (p.get_name() == "min_pixels") min_pixels_ = p.as_int();
        }
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
      });

    RCLCPP_INFO(get_logger(),
      "object_position ready. topic='%s' H=[%.1f,%.1f] S=[%.2f,%.2f] V=[%.2f,%.2f]",
      image_topic_.c_str(), hue_min_, hue_max_, sat_min_, sat_max_, val_min_, val_max_);
  }

private:
  // ---- load all parameters into member variables --------------------------
  void load_parameters()
  {
    image_topic_ = get_parameter("image_topic").as_string();
    hue_min_     = get_parameter("hue_min").as_double();
    hue_max_     = get_parameter("hue_max").as_double();
    sat_min_     = get_parameter("sat_min").as_double();
    sat_max_     = get_parameter("sat_max").as_double();
    val_min_     = get_parameter("val_min").as_double();
    val_max_     = get_parameter("val_max").as_double();
    min_pixels_  = get_parameter("min_pixels").as_int();
  }

  // ---- main callback ------------------------------------------------------
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (msg->encoding != "bgr8") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Unsupported encoding: '%s' (expected bgr8)",
                           msg->encoding.c_str());
      publish_not_found();
      return;
    }

    if (msg->data.empty() || msg->width == 0 || msg->height == 0) {
      publish_not_found();
      return;
    }

    const uint8_t *ptr = msg->data.data();

    // Accumulators for centre of gravity
    // Using double to avoid overflow on large images
    double sum_x  = 0.0;
    double sum_y  = 0.0;
    uint64_t count = 0;

    // Iterate row by row using msg->step (stride) to skip any padding bytes
    // that some drivers add at the end of each row.
    for (uint32_t row = 0; row < msg->height; ++row) {
      const uint8_t *row_ptr = ptr + row * msg->step;
      for (uint32_t col = 0; col < msg->width; ++col) {
        const uint8_t *px = row_ptr + col * 3;
        const HSV hsv = bgr_to_hsv(px[0], px[1], px[2]);

        // Check if pixel is within HSV thresholds
        const bool in_hue = (hsv.h >= static_cast<float>(hue_min_)) &&
                            (hsv.h <= static_cast<float>(hue_max_));
        const bool in_sat = (hsv.s >= static_cast<float>(sat_min_)) &&
                            (hsv.s <= static_cast<float>(sat_max_));
        const bool in_val = (hsv.v >= static_cast<float>(val_min_)) &&
                            (hsv.v <= static_cast<float>(val_max_));

        if (in_hue && in_sat && in_val) {
          // Centre of gravity: sum pixel coordinates
          // CoG_x = Σ(col) / N,  CoG_y = Σ(row) / N
          sum_x += col;
          sum_y += row;
          ++count;
        }
      }
    }

    // Require a minimum number of matching pixels to avoid noise
    if (count < static_cast<uint64_t>(min_pixels_)) {
      publish_not_found();
      return;
    }

    // Publish CoG position in pixel coordinates
    geometry_msgs::msg::Point pos_msg;
    pos_msg.x = sum_x / static_cast<double>(count);
    pos_msg.y = sum_y / static_cast<double>(count);
    pos_msg.z = 0.0;
    pos_pub_->publish(pos_msg);

    std_msgs::msg::Bool found_msg;
    found_msg.data = true;
    found_pub_->publish(found_msg);

    RCLCPP_DEBUG(get_logger(),
      "Object found at (%.1f, %.1f), pixels=%lu", pos_msg.x, pos_msg.y, count);
  }

  // ---- helper: publish not found state ------------------------------------
  void publish_not_found()
  {
    std_msgs::msg::Bool found_msg;
    found_msg.data = false;
    found_pub_->publish(found_msg);
  }

  // ---- member variables ---------------------------------------------------
  std::string image_topic_;
  double hue_min_, hue_max_;
  double sat_min_, sat_max_;
  double val_min_, val_max_;
  int    min_pixels_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr  pos_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr        found_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectPositionNode>());
  rclcpp::shutdown();
  return 0;
}
