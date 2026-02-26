from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ---- cam2image config (same as brightness_control launch) --------------
    cam_cfg = os.path.join(
        get_package_share_directory('cam2image_vm2ros'),
        'config',
        'cam2image.yaml'
    )

    return LaunchDescription([

        # ---- camera node ---------------------------------------------------
        Node(
            package='cam2image_vm2ros',
            executable='cam2image',
            name='cam2image',
            parameters=[cam_cfg],
            output='screen',
        ),

        # ---- object position node ------------------------------------------
        Node(
            package='object_position',
            executable='object_position_node',
            name='object_position',
            output='screen',
            parameters=[{
                'image_topic': '/image',
                'hue_min':  140.0,
                'hue_max':  170.0,
                'sat_min':    0.5,
                'sat_max':    1.0,
                'val_min':    0.2,
                'val_max':    0.7,
                'min_pixels': 20,
            }],
        ),
    ])