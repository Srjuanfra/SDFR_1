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
        Node(
            package='cam2image_vm2ros',
            executable='cam2image',
            name='cam2image',
            parameters=[cam_cfg],
            output='screen',
        ),
        Node(
            package='brightness_control',
            executable='brightness_control_node',
            name='brightness_control',
            parameters=[{'threshold': 80.0}],
            output='screen',
        ),
    ])
