from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='keyboard_teleop',
            executable='keyboard_teleop',
            output='screen',
        ),
    ])
