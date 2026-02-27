# Brightness Control Node

## Compile

```bash
cd ~/ros2/ros2_ws
rm -rf build install log
colcon build
source install/setup.bash
```

## Launch

```bash
ros2 launch brightness_control brightness_pipeline.launch.py
```

## Change Threshold at Launch

```bash
ros2 run brightness_control brightness_control_node --ros-args -p threshold:=150.0
```

## Change Threshold at Runtime

```bash
ros2 param set /brightness_control threshold 150.0
```

## Check Output

Is it light or dark?
```bash
ros2 topic echo /brightness/is_light
```

Average brightness value (0-255):
```bash
ros2 topic echo /brightness/avg
```
