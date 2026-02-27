#ifndef RELBOT_ADAPTER_HPP_
#define RELBOT_ADAPTER_HPP_

// CPP library headers
#include <cstdio>
#include <chrono>
#include <memory>

// ROS Client Library CPP
#include "rclcpp/rclcpp.hpp"

// Standard Message types
#include "example_interfaces/msg/float64.hpp"

// Geometry messages, used for location notation
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "relbot_msgs/msg/relbot_motors.hpp"

#ifdef HAS_XRF2_MSGS
#include "xrf2_msgs/msg/ros2_xeno.hpp"
#endif

using std::placeholders::_1;
using namespace std::chrono_literals;

/**
 * @brief RELbot adapter node.
 *
 * ROS2 parameters are described below:
 */

class RELbotAdapter : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new RELbotAdapter object
     *
     */
    RELbotAdapter();

    const std::string input = "/input";
    const std::string output = "/output";

    // Default input topic names
    const std::string TWIST_TOPIC = input + "/twist";
    const std::string SETPOINT_VEL_TOPIC = "/setpoint_vel";
    const std::string RIGHT_MOTOR_NAMESPACE = input + "/right_motor";
    const std::string LEFT_MOTOR_NAMESPACE = input + "/left_motor";

    // Default parameter values
    const double DEFAULT_WHEEL_BASE_WIDTH = 0.209; // [m] Width between the center of both wheels
    const double DEFAULT_WHEEL_RADIUS = 0.05;      // [m] Radius of the wheels
    const bool DEFAULT_USE_TWIST_CMD = false;      // Which command mode (true is Twist, false is Individual Motors)

    const double DEFAULT_GAIN = 1;

    const double UPPER_LIMIT_RADIANS_PER_SECOND = 27.5;

    const std::string DEFAULT_ROBOT_MODE = "sim";

private:
    double wheelBaseWidth_ = DEFAULT_WHEEL_BASE_WIDTH;
    double wheelRadius_ = DEFAULT_WHEEL_RADIUS;

    // Subscribers for motor commands
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twistSubscriber_;
    rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr rightMotorSetpointVelSubscriber_;
    rclcpp::Subscription<example_interfaces::msg::Float64>::SharedPtr leftMotorSetpointVelSubscriber_;

    rclcpp::Publisher<relbot_msgs::msg::RelbotMotors>::SharedPtr motor_command_sim;
    #ifdef HAS_XRF2_MSGS
    rclcpp::Publisher<xrf2_msgs::msg::Ros2Xeno>::SharedPtr motor_command_real;
    #endif
    
    // Dynamics simulation timer (plus position output)
    rclcpp::TimerBase::SharedPtr dynamics_timer_;

    // // Publisher for joint state

    rclcpp::Clock clock;

    double gain_ = DEFAULT_GAIN;
    bool useTwistCmd_ = DEFAULT_USE_TWIST_CMD;
    std::string robotmode_ = DEFAULT_ROBOT_MODE;
    double throttle_rate_ = 1.0;


    double max_speed_meters_per_second;
    double max_speed_radians_per_second;
    double max_speed_;
    double max_magnitude_;

    bool xrf2_included_ = false;

    //
    rclcpp::Time clock_start;
    int start_time;

    double storage_buffer[2];

    /**
     * @brief Create all topics for this node
     *
     */
    void create_topics();

    /**
     * @brief Callback upon receiving a Twist command velocity. Converts twist into individual rotational velocity commands to store
     * internally for the next step.
     *
     * @param twist Velocity (in rad/s and m/s) for motor to be set
     */
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr twist);

    /**
     * @brief Callback upon receiving the right motor command velocity. Stores this command internally for the next
     * step.
     *
     * @param setpoinVel setpoint velocity for right motor
     */
    void rightMotorSetpointVelCallback(const example_interfaces::msg::Float64::SharedPtr setpointVel);

    /**
     * @brief Callback upon receiving the left motor command velocity. Stores this command internally for the next
     * step.
     *
     * @param setpiontVel setpoint velocity for left motor
     */
    void leftMotorSetpointVelCallback(const example_interfaces::msg::Float64::SharedPtr setpointVel);

    /**
     * @brief Main step loop of the system. Calls the hidden Dynamics object to update to the next timestep
     *
     */
    void output_callback();

    void set_safety();
};
#endif /* RELBOT_ADAPTER_HPP_ */