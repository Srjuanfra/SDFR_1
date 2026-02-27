# object_follower

ROS2 package for the RELbot simulator that follows a coloured object detected by the webcam.

## Overview

This node subscribes to the output of the `object_position` node and computes left/right wheel velocity setpoints so that the RELbot turns to keep the detected object centred in the camera image.

**Control law (proportional):**
```
error    = image_center_x - object_x   [pixels]
turn_vel = gain_k * error              [rad/s]
left_vel  = -(base_vel - turn_vel)
right_vel =   base_vel - turn_vel
```

When the object is not found the robot stops.

## Node Graph

```
/cam2image → /image → /object_position → /object/position ─┐
                                        → /object/found    ─┤
                                                            ↓
                                            /object_follower
                                                ↑           ↓
                                    /output/robot_pose   setpoints
                                         ↑                  ↓
                                   /relbot_simulator ← /relbot_adapter
```

## Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/object/position` | `geometry_msgs/Point` | Object centre of gravity in pixel coordinates |
| `/object/found` | `std_msgs/Bool` | Whether the object is currently visible |
| `/output/robot_pose` | `geometry_msgs/PoseStamped` | Robot pose from the simulator (used for logging) |

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/input/left_motor/setpoint_vel` | `example_interfaces/Float64` | Left wheel velocity setpoint |
| `/input/right_motor/setpoint_vel` | `example_interfaces/Float64` | Right wheel velocity setpoint |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `image_width` | int | 640 | Camera image width in pixels |
| `base_vel` | double | 1.0 | Forward speed when following [rad/s] |
| `gain_k` | double | 0.01 | Proportional gain [rad/s per pixel] |
| `deadzone` | double | 20.0 | Pixel deadzone around centre — go straight within this range |

All parameters can be changed at runtime:
```bash
ros2 param set /object_follower gain_k 0.015
ros2 param set /object_follower deadzone 30.0
```

## CSV Logging

The node saves data to `/home/momta/ros2/ros2_ws/follower_data.csv` every tick (50 Hz):

```
timestamp, object_x, error, turn_vel, left_setpoint, right_setpoint, object_found, robot_x, robot_y, robot_theta
```

Useful for plotting object position vs robot position over time to verify the system works.

## Build

```bash
cd ~/ros2/ros2_ws
colcon build --packages-select object_follower
source install/setup.bash
```

## Launch

```bash
ros2 launch object_follower object_follower.launch.py
```

This launches: `cam2image`, `object_position`, `relbot_simulator`, `relbot_adapter`, and `object_follower_node`.

Adjust the HSV parameters in the launch file to match your object colour:
```python
parameters=[{
    'hue_min':  75.0,   # adjust for your object
    'hue_max': 105.0,
    'sat_min':   0.3,
    'val_min':   0.2,
}]
```

## Tuning

1. First tune the HSV parameters of `object_position` using `rqt_image_view` on `/object/mask`
2. Then adjust `gain_k` — too high causes oscillation, too low causes slow response
3. Adjust `deadzone` — larger deadzone means the robot only turns when the error is significant

## Dependencies

- `rclcpp`
- `example_interfaces`
- `geometry_msgs`
- `std_msgs`
- `object_position` package (must be running separately)