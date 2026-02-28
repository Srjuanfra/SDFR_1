//=============================================================================
// Authors     :
// Group       :
// License     : LGPL open source license
//
// Brief       : Closed-loop controller node for RELbot simulator (Assignment 1.2.3)
//
//               Implements equation 1.2 from the assignment:
//
//               x_dot_set = (1/tau) * (x_light - x_RELbot)
//               x_set     = integral(x_dot_set dt)   [Forward Euler]
//
//               Where the position of the object in the moving camera image
//               IS precisely x_light - x_RELbot:
//                 error_th = center_x - object_x  → controls theta_z (left/right)
//                 error_x  = center_y - object_y  → controls x (zoom in/out)
//
//               Forward Euler integration:
//                 th_set += (1/tau) * error_th * dt
//                 x_set  += (1/tau) * error_x  * dt
//
//               Wheel velocities (RELbot convention):
//                 left_vel  = -(x_set - th_set)
//                 right_vel =   x_set + th_set
//
// Subscribed topics:
//   /object/position   (geometry_msgs/Point) — CoG in pixels from moving camera
//   /object/found      (std_msgs/Bool)
//   /output/robot_pose (geometry_msgs/PoseStamped) — for CSV logging
//
// Published topics:
//   /input/left_motor/setpoint_vel  (example_interfaces/Float64)
//   /input/right_motor/setpoint_vel (example_interfaces/Float64)
//
// Parameters:
//   image_width   (int,    default 90)   — moving camera width in pixels
//   image_height  (int,    default 90)   — moving camera height in pixels
//   tau           (double, default 1.0)  — time constant [s]
//   max_vel       (double, default 3.0)  — max wheel velocity [rad/s]
//=============================================================================

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/float64.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <cmath>

using namespace std::chrono_literals;

class ClosedLoopFollowerNode : public rclcpp::Node
{
public:
  ClosedLoopFollowerNode() : Node("closed_loop_follower")
  {
    // ---- parameters --------------------------------------------------------
    declare_parameter<int>("image_width",  90);
    declare_parameter<int>("image_height", 90);
    declare_parameter<double>("tau",      1.0);
    declare_parameter<double>("max_vel",  3.0);

    image_width_  = get_parameter("image_width").as_int();
    image_height_ = get_parameter("image_height").as_int();
    tau_          = get_parameter("tau").as_double();
    max_vel_      = get_parameter("max_vel").as_double();

    center_x_ = image_width_  / 2.0;
    center_y_ = image_height_ / 2.0;

    // ---- publishers --------------------------------------------------------
    left_pub_  = create_publisher<example_interfaces::msg::Float64>(
      "/input/left_motor/setpoint_vel", 10);
    right_pub_ = create_publisher<example_interfaces::msg::Float64>(
      "/input/right_motor/setpoint_vel", 10);

    // ---- subscribers -------------------------------------------------------
    position_sub_ = create_subscription<geometry_msgs::msg::Point>(
      "/object/position", 10,
      std::bind(&ClosedLoopFollowerNode::positionCallback, this, std::placeholders::_1));

    found_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/object/found", 10,
      std::bind(&ClosedLoopFollowerNode::foundCallback, this, std::placeholders::_1));

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/output/robot_pose", 10,
      std::bind(&ClosedLoopFollowerNode::poseCallback, this, std::placeholders::_1));

    // ---- CSV logging -------------------------------------------------------
    csv_file_.open("/home/momta/ros2/ros2_ws/closed_loop_data.csv");
    if (csv_file_.is_open()) {
      csv_file_ << "timestamp,object_x,object_y,error_th,error_x,"
                << "th_set,x_set,left_vel,right_vel,object_found,"
                << "robot_x,robot_y,robot_theta\n";
      csv_file_.flush();
    }

    // ---- timer 50 Hz -------------------------------------------------------
    start_time_ = now();
    last_time_  = now();
    timer_ = create_wall_timer(20ms,
      std::bind(&ClosedLoopFollowerNode::timerCallback, this));

    RCLCPP_INFO(get_logger(),
      "closed_loop_follower ready. center=(%.0f,%.0f) tau=%.2f max_vel=%.2f",
      center_x_, center_y_, tau_, max_vel_);
  }

  ~ClosedLoopFollowerNode()
  {
    if (csv_file_.is_open()) {
      csv_file_.flush();
      csv_file_.close();
    }
  }

private:
  void positionCallback(const geometry_msgs::msg::Point::SharedPtr msg)
  {
    object_x_ = msg->x;
    object_y_ = msg->y;
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

  void timerCallback()
  {
    // Real dt from last tick
    const auto t_now = now();
    double dt = (t_now - last_time_).seconds();
    last_time_ = t_now;
    dt = std::clamp(dt, 0.001, 0.1);

    double left_vel  = 0.0;
    double right_vel = 0.0;

    if (object_found_) {
      // ---- Compute errors (eq. 1.2) ----------------------------------------
      // Normalize to [-1, 1] so tau is independent of image resolution
      // error_th: horizontal error → controls theta_z (turning)
      // error_x:  vertical error  → controls x (zoom in/out)
      const double error_th = (center_x_ - object_x_) / center_x_;  // [-1, 1]
      const double error_x  = (center_y_ - object_y_) / center_y_;  // [-1, 1]

      // ---- Forward Euler integration (eq. 1.2) -----------------------------
      // x_set  += (1/tau) * error_x  * dt
      // th_set += (1/tau) * error_th * dt
      x_set_  += (1.0 / tau_) * error_x  * dt;
      th_set_ += (1.0 / tau_) * error_th * dt;

      // Clamp integrator states
      x_set_  = std::clamp(x_set_,  -max_vel_, max_vel_);
      th_set_ = std::clamp(th_set_, -max_vel_, max_vel_);

      // ---- Differential drive conversion (RELbot convention) ---------------
      left_vel  = -(x_set_ - th_set_);
      right_vel =   x_set_ + th_set_;

      left_vel  = std::clamp(left_vel,  -max_vel_, max_vel_);
      right_vel = std::clamp(right_vel, -max_vel_, max_vel_);

      // Console log at ~1Hz
      log_tick_++;
      if (log_tick_ >= 50) {
        log_tick_ = 0;
        RCLCPP_INFO(get_logger(),
          "FOUND | obj=(%.1f,%.1f) | err_th=%.2f err_x=%.2f | th_set=%.3f x_set=%.3f | L=%.2f R=%.2f | robot_x=%.3fm",
          object_x_, object_y_, error_th, error_x, th_set_, x_set_, left_vel, right_vel, robot_x_);
      }

    } else {
      // Object lost — reset integrator to avoid windup
      x_set_  = 0.0;
      th_set_ = 0.0;

      log_tick_++;
      if (log_tick_ >= 50) {
        log_tick_ = 0;
        RCLCPP_INFO(get_logger(), "NOT FOUND — integrator reset, stopped");
      }
    }

    publishAndLog(left_vel, right_vel);
  }

  void publishAndLog(double left_vel, double right_vel)
  {
    example_interfaces::msg::Float64 lmsg, rmsg;
    lmsg.data = left_vel;
    rmsg.data = right_vel;
    left_pub_->publish(lmsg);
    right_pub_->publish(rmsg);

    if (csv_file_.is_open()) {
      const double ts       = (now() - start_time_).seconds();
      const double error_th = center_x_ - object_x_;
      const double error_x  = center_y_ - object_y_;
      csv_file_ << ts           << ","
                << object_x_    << ","
                << object_y_    << ","
                << error_th     << ","
                << error_x      << ","
                << th_set_      << ","
                << x_set_       << ","
                << left_vel     << ","
                << right_vel    << ","
                << object_found_ << ","
                << robot_x_     << ","
                << robot_y_     << ","
                << robot_theta_ << "\n";
    }
  }

  // ---- publishers / subscribers ------------------------------------------
  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr left_pub_;
  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr right_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr     position_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr           found_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::ofstream csv_file_;
  rclcpp::Time  start_time_;
  rclcpp::Time  last_time_;
  int           log_tick_{0};

  // Parameters
  int    image_width_{90};
  int    image_height_{90};
  double tau_{1.0};
  double max_vel_{3.0};
  double center_x_{45.0};
  double center_y_{45.0};

  // Integrator states (x_set)
  double x_set_{0.0};
  double th_set_{0.0};

  // Sensor state
  double object_x_{0.0};
  double object_y_{0.0};
  bool   object_found_{false};

  // Robot pose (logging)
  double robot_x_{0.0};
  double robot_y_{0.0};
  double robot_theta_{0.0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ClosedLoopFollowerNode>());
  rclcpp::shutdown();
  return 0;
}