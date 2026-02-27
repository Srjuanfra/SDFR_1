from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ---- cam2image config --------------------------------------------------
    cam_cfg = os.path.join(
        get_package_share_directory('cam2image_vm2ros'),
        'config',
        'cam2image.yaml'
    )

    # ---- nodes -------------------------------------------------------------

    # Camera node: streams webcam images to /image
    cam2image_node = Node(
        package='cam2image_vm2ros',
        executable='cam2image',
        name='cam2image',
        parameters=[cam_cfg],
        output='screen',
    )

    # RELbot simulator: simulates robot physics and camera
    # Subscribes to /input/motor_cmd, publishes /output/robot_pose
    relbot_simulator_node = Node(
        package='relbot_simulator',
        executable='relbot_simulator',
        name='RELbot_simulator',
        output='screen',
    )

    # RELbot adapter: validates and limits wheel velocity commands
    # Subscribes to /input/left_motor/setpoint_vel and /input/right_motor/setpoint_vel
    # Publishes to /output/motor_cmd
    relbot_adapter_node = Node(
        package='relbot_adapter',
        executable='relbot_adapter',
        name='RELbot_Adapter',
        output='screen',
        parameters=[{
            'robotmode':      'sim',
            'use_twist_cmd':  False,
            'max_speed_mps':  0.25,
            'max_speed_rads': 5.0,
        }],
        remappings=[
            ('output/motor_cmd', 'input/motor_cmd'),
        ],
    )

    # Sequence controller: publishes hardcoded square trajectory setpoints
    sequence_controller_node = Node(
        package='sequence_test',
        executable='sequence_controller_node',
        name='sequence_controller',
        output='screen',
    )

    return LaunchDescription([
        cam2image_node,
        relbot_simulator_node,
        relbot_adapter_node,
        sequence_controller_node,
    ])
