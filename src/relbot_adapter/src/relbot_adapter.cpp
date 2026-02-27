#include "relbot_adapter.hpp"

// Constructor + Setting up Topics

RELbotAdapter::RELbotAdapter() : Node("RELbot_Adapter")
{
  RCLCPP_INFO(get_logger(), "Init");
  #ifdef HAS_XRF2_MSGS
    xrf2_included_ = true;
  #endif
  RCLCPP_INFO(this->get_logger(), "xrf2 found: %d", xrf2_included_);

  create_topics();
  set_safety();

  RCLCPP_INFO(get_logger(), "Created Topics and set safety");

  dynamics_timer_ = this->create_wall_timer(std::chrono::duration<double>(), std::bind(&RELbotAdapter::output_callback, this));

  RCLCPP_INFO(get_logger(), "Created Timer");

  if (robotmode_ != "sim" && robotmode_ != "real")
  {
    RCLCPP_ERROR(this->get_logger(), "Parameter robotmode is set to an invalid value! Value: %s", robotmode_.c_str());
    exit(1);
  }
  else
  {
    if (robotmode_ == "real" && xrf2_included_ == false)
    {
      RCLCPP_WARN(this->get_logger(), "Parameter robotmode cannot be set to %s, as no XRF2 is found", robotmode_.c_str());
      robotmode_ = "sim";
    }
    RCLCPP_INFO(this->get_logger(), "Parameter robotmode is set to value: %s", robotmode_.c_str());
  }
}

void RELbotAdapter::create_topics()
{
  useTwistCmd_ = this->declare_parameter("use_twist_cmd", RELbotAdapter::DEFAULT_USE_TWIST_CMD); // default Twist is off
  gain_ = this->declare_parameter("gain", RELbotAdapter::DEFAULT_GAIN);                          // currently unused
  robotmode_ = this->declare_parameter("robotmode", RELbotAdapter::DEFAULT_ROBOT_MODE);          // Default: Run in sim-mode
  throttle_rate_ = this->declare_parameter("throttle_rate", 5000);

  RCLCPP_INFO(this->get_logger(), "Creating topics...");
  RCLCPP_INFO(this->get_logger(), "Creating Publishers");

  if (xrf2_included_ && robotmode_ == "real")
  {
  #ifdef HAS_XRF2_MSGS
  motor_command_real = this->create_publisher<xrf2_msgs::msg::Ros2Xeno>("Ros2Xeno", 1);
  RCLCPP_INFO(this->get_logger(), "Made real publisher");
  #endif
  }
  else
  {
    motor_command_sim = this->create_publisher<relbot_msgs::msg::RelbotMotors>("output/motor_cmd", 1);
    RCLCPP_INFO(this->get_logger(), "Made sim publisher");
  }

  // get param, can be set by ros2 launch command
  useTwistCmd_ = get_parameter("use_twist_cmd").as_bool();
  gain_ = get_parameter("gain").as_double();

  RCLCPP_INFO(this->get_logger(), "Creating Subscriptions");

  if (RELbotAdapter::useTwistCmd_)
  {
    RCLCPP_INFO(get_logger(), "Using Twist Command mode");

    RELbotAdapter::twistSubscriber_ = create_subscription<geometry_msgs::msg::Twist>(
        RELbotAdapter::TWIST_TOPIC, rclcpp::SensorDataQoS().reliable(),
        std::bind(&RELbotAdapter::twistCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to topic %s", RELbotAdapter::TWIST_TOPIC.c_str());
  }
  else
  {
    RCLCPP_INFO(get_logger(), "Using Individual Motors Command mode");

    // The motor topics are in a namespace, construct the full topic name first
    const std::string right_motor_setpoint_vel_topic =
        RELbotAdapter::RIGHT_MOTOR_NAMESPACE + RELbotAdapter::SETPOINT_VEL_TOPIC;
    RELbotAdapter::rightMotorSetpointVelSubscriber_ = create_subscription<example_interfaces::msg::Float64>(
        right_motor_setpoint_vel_topic, 10,
        std::bind(&RELbotAdapter::rightMotorSetpointVelCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to topic %s", right_motor_setpoint_vel_topic.c_str());

    const std::string left_motor_setpoint_vel_topic =
        RELbotAdapter::LEFT_MOTOR_NAMESPACE + RELbotAdapter::SETPOINT_VEL_TOPIC;
    RELbotAdapter::leftMotorSetpointVelSubscriber_ = create_subscription<example_interfaces::msg::Float64>(
        left_motor_setpoint_vel_topic, 10,
        std::bind(&RELbotAdapter::leftMotorSetpointVelCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Subscribed to topic %s", left_motor_setpoint_vel_topic.c_str());
  }
}

// Input from Subscriptions

void RELbotAdapter::twistCallback(const geometry_msgs::msg::Twist::SharedPtr twist)
{
  // Extract the base link velocity components from the twist
  double linearVelSetpoint = twist->linear.x * gain_;
  double angularVelSetpoint = twist->angular.z * gain_;

  double left_wheel_vel;
  double right_wheel_vel;

  // Convert the base link velocity setpoints to individual motor setpoints
  // Don't forget: everything is right hand defined, so w_L CCW = FW
  /* Hi students, code author here. Twists and reference frames are confusing and need rigid definitions. Even ChatGPT struggled with comprehending this 😃.
  Also, just using separate wheel velocities is totally okay and the originally intended way to control this sim 😉 */
  left_wheel_vel = -((linearVelSetpoint - ((angularVelSetpoint * wheelBaseWidth_) / 2)) / wheelRadius_);
  right_wheel_vel = ((linearVelSetpoint + ((angularVelSetpoint * wheelBaseWidth_) / 2)) / wheelRadius_);

  storage_buffer[0] = left_wheel_vel;
  storage_buffer[1] = right_wheel_vel;
}

void RELbotAdapter::rightMotorSetpointVelCallback(const example_interfaces::msg::Float64::SharedPtr setpointVel)
{
  storage_buffer[1] = setpointVel->data;
}

void RELbotAdapter::leftMotorSetpointVelCallback(const example_interfaces::msg::Float64::SharedPtr setpointVel)
{
  storage_buffer[0] = setpointVel->data;
}

// Timer Callback Dynamics. Calulates kinematics and dynamics

void RELbotAdapter::output_callback()
{
  // Reset to safe value
  max_magnitude_ = 0.0;

  // Fin largest number
  max_magnitude_ = std::max(std::abs(storage_buffer[0]), std::abs(storage_buffer[1]));

  // Some safety for the motor speeds
  if (max_magnitude_ > max_speed_ && max_magnitude_ > 0.0)
  {
    // find scaling value
    double scaling_factor = max_speed_ / max_magnitude_;

    // rescale BOTH components to keep same directionality
    storage_buffer[0] = storage_buffer[0] * scaling_factor;
    storage_buffer[1] = storage_buffer[1] * scaling_factor;

    RCLCPP_INFO_THROTTLE(this->get_logger(), clock, throttle_rate_, "Value to large, scaling back to maximum motor velocities");
  }

  if (xrf2_included_ && robotmode_ == "real") {

  #ifdef HAS_XRF2_MSGS
      xrf2_msgs::msg::Ros2Xeno motor_output;

      // motor_output.header.stamp = get_clock()->now();
      motor_output.set__left_wheel_vel(storage_buffer[0]);
      motor_output.set__right_wheel_vel(storage_buffer[1]);

      RCLCPP_INFO_THROTTLE(this->get_logger(), clock, throttle_rate_, "Output Real = [L,R] = [%f, %f]", storage_buffer[0], storage_buffer[1]);

      // Output the actual position
      motor_command_real->publish(motor_output);
  #endif
  }
  else
  {
    relbot_msgs::msg::RelbotMotors motor_output;

    // motor_output.header.stamp = get_clock()->now();
    motor_output.set__left_wheel_vel(storage_buffer[0]);
    motor_output.set__right_wheel_vel(storage_buffer[1]);

    // Output the actual position
    motor_command_sim->publish(motor_output);
  }
}

void RELbotAdapter::set_safety()
{
  max_speed_meters_per_second = this->declare_parameter("max_speed_mps", 0.25);
  max_speed_radians_per_second = this->declare_parameter("max_speed_rads", 5.0);

  max_speed_ = 0;
  double rads_from_ms = max_speed_meters_per_second / 0.05;

  // Clamp values to upper limit if they’re above it
  double from_meters = std::clamp(rads_from_ms, 0.0, UPPER_LIMIT_RADIANS_PER_SECOND);
  double from_radians = std::clamp(max_speed_radians_per_second, 0.0, UPPER_LIMIT_RADIANS_PER_SECOND);

  // Determine if user changed either input
  bool meters_changed = max_speed_meters_per_second != 0.25;
  bool radians_changed = max_speed_radians_per_second != 5.0;

  if (meters_changed && !radians_changed)
  {
    max_speed_ = from_meters;
    RCLCPP_INFO(this->get_logger(), "Only meters per second was changed; using %.2f rad/s (converted from %.2f m/s)", from_meters, max_speed_meters_per_second);
  }
  else if (!meters_changed && radians_changed)
  {
    max_speed_ = from_radians;
    RCLCPP_INFO(this->get_logger(), "Only radians per second was changed; using %.2f rad/s", from_radians);
  }
  else if (from_meters > 0 || from_radians > 0)
  {
    max_speed_ = std::min(from_meters, from_radians);
    RCLCPP_INFO(this->get_logger(), "%s were changed or unchanged; using conservative minimum of %.2f rad/s", (meters_changed && radians_changed) ? "Both inputs" : "Neither input", max_speed_);
  }
  else
  {
    RCLCPP_WARN(this->get_logger(),
                "Invalid or zero speeds given; setting max to default of 5.0 rad/s");
    max_speed_ = 5.0;
  }
  RCLCPP_INFO(this->get_logger(), "Max speed set to = %f in rad/s, or %f in m/s", max_speed_, max_speed_ * 0.05);
}

int main(int argc, char *argv[])
{
  printf("RELbot Adapter Node\n-------------------\n");
  rclcpp::init(argc, argv);
  auto RELbot_adapter= std::make_shared<RELbotAdapter>();
  rclcpp::spin(RELbot_adapter);
  printf("Done Spinning\n-------------------\n");
  rclcpp::shutdown();
  return 0;
}
