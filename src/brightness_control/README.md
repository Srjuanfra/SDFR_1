To compile first:
    cd ~/ros2/ros2_ws
    rm -rf build install log
    colcon build
    source install/setup.bash

To run it:
    ros2 launch brightness_control brightness_pipeline.launch.py

To change the Threshold (Set the value as u want it):
    ros2 run brightness_control brightness_control_node --ros-args -p threshold:=150.0 
