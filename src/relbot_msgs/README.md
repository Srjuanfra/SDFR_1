Package relbot_msgs
-----------------------------------------------
### Description: 
This package defines the `relbot_msgs`, used for specific combined commands towards the relbot_sim

### Definitions:

`relbot_msgs/msg/RelbotMotors`  
Defines a `left_wheel_vel` and `right_wheel_vel` to drive the motors. Singular message as to always have two values if needed. Prevents command delays possibly affecting motor activation moment.

`relbot_msgs/msg/RelbotMotorsStamped`  
Same as the normal one, but with timestamp capabilities in the `header` field

