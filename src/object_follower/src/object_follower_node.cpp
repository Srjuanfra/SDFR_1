//=============================================================================
// Authors     :
// Group       :
// License     : LGPL open source license
//
// Brief       : Object follower node for the RELbot simulator.
//               Subscribes to /object/position and /object/found from the
//               object_position node and computes left/right wheel velocity
//               setpoints so that the RELbot turns to keep the detected
//               object centred in the camera image.
//
//               Control law (proportional):
//                 error     = image_center_x - object_x   [pixels]
//                 turn_vel  = K * error                   [rad/s]
//                 left_vel  = -(BASE_VEL - turn_vel)
//                 right_vel =   BASE_VEL - turn_vel
//
//               When the object is not found the robot stops.
//
// Subscribed topics:
//   /object/position  (geometry_msgs/Point)  — object CoG in pixel coords
//   /object/found     (std_msgs/Bool)         — whether object is visible
//
// Published topics:
//   /input/left_motor/setpoint_vel   (example_interfaces/Float64)
//   /input/right_motor/setpoint_vel  (example_interfaces/Float64)
//
// Parameters (settable at runtime via ros2 param set):
//   image_width   (int,    default 640)   — camera image width in pixels
//   base_vel      (double, default 1.0)   — forward speed when following [rad/s]
//   gain_k        (double, default 0.01)  — proportional gain [rad/s per pixel]
//   deadzone      (double, default 20.0)  — pixels around centre to go straight
//=============================================================================

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/float64.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <chrono>
#include <fstream>
#include <cmath>

using namespace std::chrono_literals;

class ObjectFollowerNode : public rclcpp::Node
{
public:
  ObjectFollowerNode() : Node("object_follower")
  {
    // ---- declare parameters ------------------------------------------------
    declare_parameter<int>("image_width", 640);
    declare_parameter<double>("base_vel",  1.0);
    declare_parameter<double>("gain_k",    0.01);
    declare_parameter<double>("deadzone",  20.0);

    image_width_ = get_parameter("image_width").as_int();
    base_vel_    = get_parameter("base_vel").as_double();
    gain_k_      = get_parameter("gain_k").as_double();
    deadzone_    = get_parameter("deadzone").as_double();

    // ---- publishers --------------------------------------------------------
    left_pub_  = create_publisher<example_interfaces::msg::Float64>(
      "/input/left_motor/setpoint_vel", 10);
    right_pub_ = create_publisher<example_interfaces::msg::Float64>(
      "/input/right_motor/setpoint_vel", 10);

    // ---- subscribers -------------------------------------------------------
    position_sub_ = create_subscription<geometry_msgs::msg::Point>(
      "/object/position", 10,
      std::bind(&ObjectFollowerNode::positionCallback, this, std::placeholders::_1));

    found_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/object/found", 10,
      std::bind(&ObjectFollowerNode::foundCallback, this, std::placeholders::_1));

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/output/robot_pose", 10,
      std::bind(&ObjectFollowerNode::poseCallback, this, std::placeholders::_1));

    // ---- CSV logging -------------------------------------------------------
    csv_file_.open("/home/momta/ros2/ros2_ws/follower_data.csv");
    if (csv_file_.is_open()) {
      csv_file_ << "timestamp,object_x,error,turn_vel,left_setpoint,right_setpoint,object_found,robot_x,robot_y,robot_theta\n";
      csv_file_.flush();
    }

    // ---- control timer at 50 Hz -------------------------------------------
    start_time_ = now();
    timer_ = create_wall_timer(20ms,
      std::bind(&ObjectFollowerNode::timerCallback, this));

    RCLCPP_INFO(get_logger(),
      "object_follower ready. image_width=%d base_vel=%.2f gain_k=%.4f deadzone=%.1f",
      image_width_, base_vel_, gain_k_, deadzone_);
  }

  ~ObjectFollowerNode()
  {
    if (csv_file_.is_open()) {
      csv_file_.flush();
      csv_file_.close();
    }
  }

private:
  // ---- callbacks ---------------------------------------------------------
  void positionCallback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    object_x_ = msg->x;
  }

  void foundCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    object_found_ = msg->data;
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    robot_x_     = msg->pose.position.x;
    robot_y_     = msg->pose.position.y;
    robot_theta_ = msg->pose.orientation.z;
  }

  // ---- control loop ------------------------------------------------------
  void timerCallback()
  {
    double left_vel  = 0.0;
    double right_vel = 0.0;
    double error     = 0.0;
    double turn_vel  = 0.0;

    if (object_found_) {
      // Error: positive = object is to the left of centre
      //        negative = object is to the right of centre
      const double center_x = image_width_ / 2.0;
      error = center_x - object_x_;

      // Apply deadzone — go straight if object is close to centre
      if (std::fabs(error) < deadzone_) {
        error = 0.0;
      }

      turn_vel  = gain_k_ * error;

      // RELbot convention from the sequence controller:
      //   straight: left = -DRIVE_VEL, right = +DRIVE_VEL
      //   turn:     both same sign
      // So to go forward and steer:
      left_vel  = -(base_vel_ - turn_vel);
      right_vel =   base_vel_ - turn_vel;
    }
    // If object not found: left_vel = right_vel = 0.0 (stop)

    // Console logging at ~1Hz (every 50 ticks at 50Hz)
    log_tick_++;
    if (log_tick_ >= 50) {
      log_tick_ = 0;
      if (object_found_) {
        RCLCPP_INFO(get_logger(),
          "Object FOUND | object_x=%.1f | error=%.1f | robot_x=%.3f m",
          object_x_, error, robot_x_);
      } else {
        RCLCPP_INFO(get_logger(), "Object NOT FOUND — stopped");
      }
    }

    publishAndLog(left_vel, right_vel, error, turn_vel);
  }

  // ---- publish and log ---------------------------------------------------
  void publishAndLog(double left_vel, double right_vel,
                     double error, double turn_vel)
  {
    example_interfaces::msg::Float64 left_msg;
    example_interfaces::msg::Float64 right_msg;
    left_msg.data  = left_vel;
    right_msg.data = right_vel;
    left_pub_->publish(left_msg);
    right_pub_->publish(right_msg);

    if (csv_file_.is_open()) {
      const double ts = (now() - start_time_).seconds();
      csv_file_ << ts           << ","
                << object_x_    << ","
                << error        << ","
                << turn_vel     << ","
                << left_vel     << ","
                << right_vel    << ","
                << object_found_ << ","
                << robot_x_     << ","
                << robot_y_     << ","
                << robot_theta_ << "\n";
    }
  }

  // ---- members -----------------------------------------------------------
  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr left_pub_;
  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr right_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr     position_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr           found_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr                                   timer_;

  std::ofstream csv_file_;
  rclcpp::Time  start_time_;

  // Parameters
  int    image_width_;
  double base_vel_;
  double gain_k_;
  double deadzone_;

  // State
  double object_x_     = 0.0;
  bool   object_found_ = false;
  double robot_x_      = 0.0;
  double robot_y_      = 0.0;
  double robot_theta_  = 0.0;
  int    log_tick_     = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectFollowerNode>());
  rclcpp::shutdown();
  return 0;
}