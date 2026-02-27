#ifndef RELBOT_SIMULATOR_HPP_
#define RELBOT_SIMULATOR_HPP_

// CPP library headers
#include <cstdio>
#include <chrono>
#include <memory>

// ROS Client Library CPP
#include "rclcpp/rclcpp.hpp"

// Sensor message types related to images
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/fill_image.hpp"

// Geometry messages, used for location notation
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

// OpenCV imshow
#include "opencv2/core/mat.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"

#if __has_include(<cv_bridge/cv_bridge.hpp>)
    #include <cv_bridge/cv_bridge.hpp>
#elif __has_include(<cv_bridge/cv_bridge.h>)
    #include <cv_bridge/cv_bridge.h>
#else
    #error "Required cv_bridge header file not found"
#endif

// Motor messages, used for moter speed input
#include "relbot_msgs/msg/relbot_motors.hpp"
// Self written headers
#include "Plant.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

/**
 * @brief RELbot simulator node.
 *
 * ROS2 parameters are described below:
 * @param use_twist_cmd [bool]: Which command mode to use: true is Twist, false is Individual Motors. Default is false.
 * @param image_stream_FPS [double]: How often should the simulator node publish a new frame? Default: 30
 */

class RELbotSimulator : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new RELbotSimulator object
   *
   * @param time_step time-step defines how quickly the motors are simulated to catch up to commands.
   */
  RELbotSimulator(double time_step);

  const std::string input = "/input";
  const std::string output = "/output";

  // Default input topic names
  
  const std::string SETPOINT_VEL_TOPIC = input + "/motor_cmd";
  const std::string WEBCAM_IMAGE = "/image";

  // Default ouput topic names
  const std::string CAMERA_POSITION = output + "/camera_position";
  const std::string ROBOT_POSE = output + "/pose";

  // Default parameter values
  const double DEFAULT_WHEEL_BASE_WIDTH = 0.209; // [m] Width between the center of both wheels
  const double DEFAULT_WHEEL_RADIUS = 0.05;      // [m] Radius of the wheels
  const double DEFAULT_TIME_STEP = 0.001;        // [s] Time step of the simulation
  const double DEFAULT_TIME_CONSTANT = 0.3;      // [s] Time constant to make motor velocities behave as 1st order system
  const bool DEFAULT_USE_TWIST_CMD = false;      // Which command mode (true is Twist, false is Individual Motors)
  const double DEFAULT_IMAGE_STREAM_FPS = 30;

  const double RADS_TO_DUTY_CYCLE = 27.5;

  double wheelBaseWidth_ = DEFAULT_WHEEL_BASE_WIDTH;
  double wheelRadius_ = DEFAULT_WHEEL_RADIUS;

private:
  // Topics
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr webcam_input_topic_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr moving_camera_output_topic_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr camera_position_topic_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_topic;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtlesim_visualisation;

 
  // Subscribers for motor commands
  rclcpp::Subscription<relbot_msgs::msg::RelbotMotors>::SharedPtr motor_velocity_Subscriber_;

  // // Publisher for joint state

  // Dynamics simulation timer (plus position output)
  rclcpp::TimerBase::SharedPtr dynamics_timer_;
  rclcpp::TimerBase::SharedPtr image_stream_timer_;
  rclcpp::TimerBase::SharedPtr visualisation_output_timer_;

  Plant _20Sim_rt_sim = Plant();

  // Internal private input states
  XXDouble input_vector[2];
  XXDouble output_vector[5];

  rclcpp::Clock clock;

  // Parameter storage and init
  bool useTwistCmd_ = DEFAULT_USE_TWIST_CMD;

  double image_stream_FPS = DEFAULT_IMAGE_STREAM_FPS;
  double throttle_rate_ = 1.0;
  double visualisation_frequency = 62.5;

  // Storage for relbout output position to twist calculation
  // positions used in calculations
  float x = 0;
  float y = 0;
  float theta = 0;
  int seconds = 0;
  uint nanosec = 0;
  
  // previous positions
  float prev_x = 0;
  float prev_y = 0;
  float prev_theta = 0;
  int prev_seconds = 0;
  uint prev_nanosec = 0;

  // speed to send to turtlesim
  float x_speed_fixed;
  float theta_speed_fixed;


  // Container for building the output image as a message type.
  cv::Mat output_image_;
  std::shared_ptr<sensor_msgs::msg::Image> sensor_output_img;

  sensor_msgs::msg::Image::SharedPtr input_image;
  //
  rclcpp::Time clock_start;
  int start_time;

  /**
   * @brief Create all topics for this node
   *
   */
  void create_topics();

  /**
   * @brief Handles receiving of received webcam images
   *
   * @param msg_cam_img webcam image, in sensor_msgs::msg::Image format
   */
  void webcam_topic_callback(const sensor_msgs::msg::Image::SharedPtr msg_cam_img);

  /**
   * @brief Callback for receiving the motor velocity
   * 
   * @param motor_velocity 
   */
  void motorVelocityCallback(const relbot_msgs::msg::RelbotMotors motor_velocity_cmd);

  // Timers

  /**
   * @brief Main step loop of the system. Calls the hidden Dynamics object to update to the next timestep
   *
   */
  void dynamics_timer_callback();

    /**
   * @brief Secondary loop of the system, which outputs the achieved system movement to a Twist such that it can be used for visualisation
   *
   */
  void visualisation_output_timer_callback();

  // Image handling

  /**
   * @brief Consitently outputs the webcam stream of the system. Runs at 30 FPS by default, can be set by ROS param
   *
   */
  void image_stream_timer_callback();

  /**
   * @brief Image manipulation method to change webcam image into sub-image which is returned. Internally uses openCV methods
   *
   * @param msg_cam_img webcam image in ROS sensor format
   * @param center_pixel_x center pixel of the view along the x-axis (pan)
   * @param center_pixel_y center pixel of the view along the y-axis (height)
   * @param output_image_dim size of output image (in pixels), will not deterimine window size!
   * @return cv::Mat Used to turn back into sensor_img for between-ros communication
   */
  cv::Mat CreateCVSubimage(const sensor_msgs::msg::Image::SharedPtr msg_cam_img, const int center_pixel_x, const int center_pixel_y, int output_image_dim);
};
#endif /* RELBOT_SIMULATOR_HPP_ */