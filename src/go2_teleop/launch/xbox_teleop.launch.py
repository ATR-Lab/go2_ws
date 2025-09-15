from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='go2_teleop',
            executable='teleop_xbox_controller',
            output='screen',
        ),
    ])
