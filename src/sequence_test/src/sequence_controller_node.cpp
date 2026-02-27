#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/float64.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <chrono>
#include <fstream>
#include <cmath>

using namespace std::chrono_literals;

class SequenceControllerNode : public rclcpp::Node
{
public:
  SequenceControllerNode() 
  : Node("sequence_controller"), segment_initialized_(false)
  {
    left_pub_  = create_publisher<example_interfaces::msg::Float64>("/input/left_motor/setpoint_vel", 10);
    right_pub_ = create_publisher<example_interfaces::msg::Float64>("/input/right_motor/setpoint_vel", 10);

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/output/robot_pose", 10,
      std::bind(&SequenceControllerNode::poseCallback, this, std::placeholders::_1));

    csv_file_.open("/home/momta/ros2/ros2_ws/sequence_data.csv");

    // 50 Hz control loop
    timer_ = create_wall_timer(20ms, std::bind(&SequenceControllerNode::timerCallback, this));

    start_time_ = now();

    RCLCPP_INFO(get_logger(), "Sequence Controller Node Started. Goal: 1.0 m Square.");
  }

  ~SequenceControllerNode() 
  { 
    if (csv_file_.is_open()) 
      csv_file_.close(); 
  }

private:

  // ---- ADJUSTED PARAMETERS ----
  static constexpr double DRIVE_VEL    = 2.0;    // Constant velocity to clearly observe acceleration
  static constexpr double TURN_VEL     = 1.0;    // Slightly higher turn speed to overcome friction
  static constexpr double TARGET_DIST  = 0.10;   // Target distance per side 
  static constexpr double TARGET_ANGLE = 1.5708; // 90 degrees (PI/2)
  static constexpr double PUBLISH_RATE_HZ = 50.0;
  static constexpr int STARTUP_TICKS = 150; 

  int turn_timeout_ticks_ = 0;

  enum class State { FORWARD, TURN, STOP };
  State state_ = State::FORWARD;

  int side_ = 0;
  int startup_ticks_ = 0;
  bool segment_initialized_;

  double pose_x_ = 0.0;
  double pose_y_ = 0.0;
  double theta_z_ = 0.0;

  double start_x_ = 0.0;
  double start_y_ = 0.0;
  double start_theta_ = 0.0;

  rclcpp::Time start_time_;
  std::ofstream csv_file_;

  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr left_pub_;
  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr right_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    pose_x_ = msg->pose.position.x;
    pose_y_ = msg->pose.position.y;
    theta_z_ = msg->pose.orientation.z; 
  }

  void timerCallback()
  {
    // Startup delay to allow simulator stabilization
    if (startup_ticks_ < STARTUP_TICKS)
    {
      startup_ticks_++;
      publishAndLog(0.0, 0.0);
      return;
    }

    if (state_ == State::STOP)
      return;

    if (!segment_initialized_)
    {
      start_x_ = pose_x_;
      start_y_ = pose_y_;
      start_theta_ = theta_z_;
      segment_initialized_ = true;
      turn_timeout_ticks_ = 0;

      RCLCPP_INFO(
        get_logger(),
        "Segment %d: X=%.2f, Y=%.2f, Angle=%.2f",
        side_ + 1,
        start_x_,
        start_y_,
        start_theta_);
    }

    if (state_ == State::FORWARD)
    {
      double dist = std::sqrt(
        std::pow(pose_x_ - start_x_, 2) +
        std::pow(pose_y_ - start_y_, 2));

      if (dist >= TARGET_DIST)
      {
        state_ = State::TURN;
        segment_initialized_ = false;
        publishAndLog(0.0, 0.0);
      }
      else
      {
        // Remember To move straight in RELbot, wheel velocities must have opposite signs
        publishAndLog(-DRIVE_VEL, DRIVE_VEL);
      }
    }
    else if (state_ == State::TURN)
    {
      double angle_diff = std::abs(theta_z_ - start_theta_);

      if (angle_diff > M_PI)
        angle_diff = std::abs(angle_diff - 2.0 * M_PI);

      if (angle_diff >= (TARGET_ANGLE - 0.05))
      {
        side_++;
        segment_initialized_ = false;

        state_ = (side_ >= 4) ? State::STOP : State::FORWARD;

        publishAndLog(0.0, 0.0);
      }
      else
      {
        
        publishAndLog(TURN_VEL, TURN_VEL);
      }
    }
  }

  void publishAndLog(double left_val, double right_val)
  {
    auto l_msg = example_interfaces::msg::Float64();
    auto r_msg = example_interfaces::msg::Float64();

    l_msg.data = left_val;
    r_msg.data = right_val;

    left_pub_->publish(l_msg);
    right_pub_->publish(r_msg);

    if (csv_file_.is_open())
    {
      double ts = (this->now() - start_time_).seconds();

      csv_file_ << ts << ","
                << left_val << ","
                << right_val << ","
                << pose_x_ << ","
                << pose_y_ << ","
                << theta_z_ << "\n";
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SequenceControllerNode>());
  rclcpp::shutdown();
  return 0;
}