# sequence_test

## Description

Sequence controller node for the RELbot. Publishes a hardcoded square
trajectory as left/right wheel velocity setpoints to the RELbot Adapter.

The sequence consists of 4 straight segments and 4 turns (90° each),
producing a square path in the RELbot Simulator.

## Build

```bash
cd ~/ros2/ros2_ws

# Build messages first
colcon build --packages-select relbot_msgs
source install/setup.bash

# Build everything
colcon build
source install/setup.bash
```

## Launch

```bash
ros2 launch sequence_test sequence_test.launch.py
```

## Plot setpoint vs actual position (RQt)

In a separate terminal:

```bash
rqt
```

Go to `Plugins → Visualization → Plot` and add:

```
/input/left_motor/setpoint_vel/data    ← left wheel setpoint
/input/right_motor/setpoint_vel/data   ← right wheel setpoint
/output/robot_pose/pose/position/x     ← actual x position
/output/robot_pose/pose/position/y     ← actual y position
```

## Tune the square trajectory

Edit these constants in `src/sequence_controller_node.cpp`:

| Constant | Default | Description |
|---|---|---|
| `DRIVE_VEL` | 3.0 rad/s | Wheel speed while driving straight |
| `TURN_VEL` | 2.0 rad/s | Wheel speed while turning |
| `FORWARD_DURATION` | 3.0 s | Duration of each straight segment |
| `TURN_DURATION` | 1.65 s | Duration of each 90° turn |

## Check output

```bash
ros2 topic echo /output/robot_pose
ros2 topic echo /input/left_motor/setpoint_vel
ros2 topic echo /input/right_motor/setpoint_vel
```
