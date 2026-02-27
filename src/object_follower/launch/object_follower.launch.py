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

        # Webcam stream
        Node(
            package='cam2image_vm2ros',
            executable='cam2image',
            name='cam2image',
            parameters=[cam_cfg],
            output='screen',
        ),

        # Object detection — adjust HSV params for your ball
        Node(
            package='object_position',
            executable='object_position_node',
            name='object_position',
            output='screen',
            parameters=[{
                'image_topic': '/image',
                'hue_min':  80.0,
                'hue_max': 160.0,
                'sat_min':   0.3,
                'sat_max':   1.0,
                'val_min':   0.2,
                'val_max':   1.0,
                'min_pixels': 20,
            }],
        ),

        # RELbot simulator
        Node(
            package='relbot_simulator',
            executable='relbot_simulator',
            name='relbot_simulator',
            output='screen',
        ),

        # RELbot adapter (velocity safety limits)
        Node(
            package='relbot_adapter',
            executable='relbot_adapter',
            name='relbot_adapter',
            output='screen',
            remappings=[('output/motor_cmd', 'input/motor_cmd')]
        ),

        # Object follower controller
        Node(
            package='object_follower',
            executable='object_follower_node',
            name='object_follower',
            output='screen',
            parameters=[{
                'image_width': 320,
                'base_vel':    2.0,
                'gain_k':      0.02,
                'deadzone':    20.0,
            }],
        ),
    ])