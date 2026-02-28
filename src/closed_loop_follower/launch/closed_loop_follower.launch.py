from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    cam_cfg = os.path.join(
        get_package_share_directory('cam2image_vm2ros'),
        'config',
        'cam2image.yaml'
    )

    return LaunchDescription([

        # Webcam stream (steady camera, input to simulator)
        Node(
            package='cam2image_vm2ros',
            executable='cam2image',
            name='cam2image',
            parameters=[cam_cfg],
            output='screen',
        ),

        # RELbot simulator
        Node(
            package='relbot_simulator',
            executable='relbot_simulator',
            name='relbot_simulator',
            output='screen',
        ),

        # RELbot adapter
        Node(
            package='relbot_adapter',
            executable='relbot_adapter',
            name='relbot_adapter',
            output='screen',
            remappings=[('output/motor_cmd', 'input/motor_cmd')]
        ),

        # Object detection on MOVING camera output
        Node(
            package='object_position',
            executable='object_position_node',
            name='object_position',
            output='screen',
            parameters=[{
                'image_topic': '/output/moving_camera',
                'hue_min':  85.0,
                'hue_max': 150.0,
                'sat_min':   0.3,
                'sat_max':   1.0,
                'val_min':   0.2,
                'val_max':   0.8,
                'min_pixels': 5,
            }],
        ),

        # Closed loop controller
        Node(
            package='closed_loop_follower',
            executable='closed_loop_follower_node',
            name='closed_loop_follower',
            output='screen',
            parameters=[{
                'image_width':  90,
                'image_height': 90,
                'tau':          1.0,   # higher tau = slower integration, less saturation
                'max_vel':      3.0,
            }],
        ),
    ])