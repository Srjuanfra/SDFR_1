
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include <cstdint>
#include <string>

class BrightnessControlNode : public rclcpp::Node
{
public:
  BrightnessControlNode() : Node("brightness_control")
  {
    declare_parameter<std::string>("image_topic", "/image");
    declare_parameter<double>("threshold", 80.0);          // 0..255
    declare_parameter<bool>("publish_average", true);

    image_topic_ = get_parameter("image_topic").as_string();
    threshold_   = get_parameter("threshold").as_double();
    publish_avg_ = get_parameter("publish_average").as_bool();

    state_pub_ = create_publisher<std_msgs::msg::Bool>("/brightness/is_light", 10);
    if (publish_avg_) {
      avg_pub_ = create_publisher<std_msgs::msg::Float32>("/brightness/avg", 10);
    }

    sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&BrightnessControlNode::imageCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "brightness_control ready. image_topic='%s', threshold=%.1f (expects bgr8)",
      image_topic_.c_str(), threshold_);
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (msg->encoding != "bgr8") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Unsupported encoding: '%s' (expected 'bgr8')",
                           msg->encoding.c_str());
      return;
    }

    const auto &data = msg->data;
    if (data.empty() || msg->width == 0 || msg->height == 0) return;

    // bgr8 => 3 bytes per pixel
    const uint32_t bytes_per_pixel = 3;
    const uint32_t expected_step = msg->width * bytes_per_pixel;
    if (msg->step != expected_step) {
      // Not fatal, but indicates unusual padding/stride
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Unexpected step: step=%u expected=%u",
                           msg->step, expected_step);
    }

    // Compute average brightness using approximate luma:
    // Y = (29*B + 150*G + 77*R) >> 8   (approximation of 0.114, 0.587, 0.299)
    // This produces Y in range 0..255 without using floats per pixel.
    uint64_t sumY = 0;
    uint64_t count = 0;

    const uint8_t *ptr = data.data();
    const size_t total = data.size();

    // Iterate through bytes in triples (B, G, R)
    for (size_t i = 0; i + 2 < total; i += 3) {
      const uint32_t B = ptr[i + 0];
      const uint32_t G = ptr[i + 1];
      const uint32_t R = ptr[i + 2];
      const uint32_t Y = (29u * B + 150u * G + 77u * R) >> 8;
      sumY += Y;
      count++;
    }

    if (count == 0) return;

    const float avg = static_cast<float>(sumY) / static_cast<float>(count);
    const bool is_light = avg >= static_cast<float>(threshold_);

    std_msgs::msg::Bool state_msg;
    state_msg.data = is_light;
    state_pub_->publish(state_msg);

    if (publish_avg_ && avg_pub_) {
      std_msgs::msg::Float32 avg_msg;
      avg_msg.data = avg;
      avg_pub_->publish(avg_msg);
    }
  }

  std::string image_topic_;
  double threshold_;
  bool publish_avg_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr avg_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BrightnessControlNode>());
  rclcpp::shutdown();
  return 0;
}
