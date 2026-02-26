Luanch the nodes: 

ros2 launch object_position object_position.launch.py

Change parameters in the runtime:

ros2 param set /object_position hue_min 40.0

To check if there is a object:

ros2 topic echo /object/found

To check the coordinates:

ros2 topic echo /object/position

