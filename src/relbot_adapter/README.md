Package relbot_adapter
-----------------------------------------------
### Description: 
This package implements an adapter to connect to either the real of simulated relbot.

### Functionality:
Package creates a standardised adapter/adapter to communicate with `relbot_sim` or real RELbot. Package handles checking if communicating with real RELbot is possible, otherwise defaults to the sim state.
Package clips incoming commands to within default limits (0.25 m/s or 5 rad/s). Limits can be expressed in m/s or rad/s (for the wheels) and will adapt whichever of the two values evaluates to be lower.

### Build instructions:
Mostly the same as other packages, but with some quirks due to the optional use of XRF2. 

To add `XRF2`:
 - Build the `xrf2_msgs` package
 - Run `colcon build --cmake-clean-cache` to detect the change. The code should auto-detect that `XRF2` can now be used

To remove `XRF2` support:
 - Delete `xrf2_msgs` under the workspace install folder.
 - Run `colcon build` as normal.


### Inputs:
`/input/left_motor/setpoint_vel`  
`/input/right_motor/setpoint_vel`  
        Type: example_interfaces/msg/Float64

If use_twist_cmd = true:  
`/input/twist`  
        Type: geometry_msgs/msg/Twist

### Outputs:
If parameter `robotmode` = sim:  
`/output/motor_cmd`  
        Type: relbot_msgs/msg/RelbotMotors

If parameter `robotmode` = real (and XRF2 is found):  
`/Ros2Xeno`  
        Type: xrf2_msgs/msg/Ros2Xeno

### Parameters:

- `max_speed_mps`: Max speed in meters per second. Type: double, default: 0.25
- `max_speed_rads`: Max speed in radians per second. Type: double, default: 5.0
  - Both max speeds are evaluated together. Lowest will be applied. See status message on startup
- `robotmode`: either "real" or "sim". Type: string, default: sim
  - Using "real" without availability of `XRF2` will default this value to "sim"
- `use_twist_cmd`: Use twist as input. Type: bool, default: false
- `throttle_rate`: Defines how often a message can be output. First message after this time (in ms) is passed get published again, after which the wait period starts again.

### Running the node:
Default behaviour:
`
ros2 run relbot_adapter relbot_adapter
`


With all parameters being set from terminal:
```
ros2 run relbot_adapter relbot_adapter --ros-args -p use_twist_cmd:=true -p max_speed_mps:=0.5 -p max_speed_rads:=10.123456789 -p robotmode:="real"
```