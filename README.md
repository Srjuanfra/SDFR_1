# SDFR_1
# RELbot Closed-Loop Controller - ASDfR Assignment 1

This repository contains the ROS2 Jazzy implementation for the RELbot simulator project as part of the Advanced Software Development for Robotic Applications course at the University of Twente.

## Prerequisites
- **OS**: Ubuntu 24.04 (Noble Numbat)
- **Middleware**: ROS2 Jazzy Jalisco
- **Dependencies**: 
  - `relbot_simulator`
  - `cv_bridge` 
  - `sensor_msgs`, `geometry_msgs`, and `std_msgs`

## Setup and Installation
1. Move to your `colcon` workspace `src` folder:
   ```bash
   cd ~/ros2_ws/src
## Clone the repository 
  git clone <repository_url> 
  
## Buildpackages
cd ~/ros2_ws
colcon build --packages-select <your_package_name>
source install/setup.bash

## Execution 
   ros2 launch <your_package_name> <your_launch_file>.launch.py

   ## System Architecture
The system uses a modular feedback pipeline to process visual data and drive the robot actuators.



### 1. Nodes
| Node Name | Responsibility |
| :--- | :--- |
| **`brightness_control`** | Filters raw camera input and manages threshold parameters. |
| **`object_position`** | Detects the target using HSV filtering and CoG calculation. |
| **`closed_loop_follower`** | Main controller implementing the first-order control law. |

### 2. ROS2 Topics
| Topic Name | Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| `/output/moving_camera` | `sensor_msgs/Image` | **Sub** | Feedback loop from the robot's camera. |
| `/object/position` | `geometry_msgs/Point` | **Sub** | Real-time (x,y) coordinates of the light source. |
| `/input/motor_cmd` | `geometry_msgs/Twist` | **Pub** | Velocity commands for navigation. |
| `/input/left_motor/setpoint_vel` | `std_msgs/Float64` | **Pub** | Individual wheel speed control. |

### 3. Control Parameters
- **Time Constant ($\tau$):** 1.0s (Adjustable via ROS2 parameters).
- **Target Center:** 45 px (Horizontal and Vertical).
- **QoS Profile:** Reliability: `Best Effort` | History: `Keep Last` (Depth 1).
  
  
