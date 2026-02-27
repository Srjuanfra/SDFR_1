# Object Position Node

## Launch

```bash
ros2 launch object_position object_position.launch.py

## Change Parameters at Runtime

```bash
ros2 param set /object_position hue_min 75.0
ros2 param set /object_position hue_max 105.0
ros2 param set /object_position sat_min 0.3
ros2 param set /object_position val_min 0.2
```

## Check Output

Is the object detected?
```bash
ros2 topic echo /object/found
```

Where is the object? (pixel coordinates)
```bash
ros2 topic echo /object/position
```
