from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import LogInfo

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="WARNING: Keyboard teleop requires direct terminal access."),
        LogInfo(msg="If this fails, run directly: ros2 run go2_teleop teleop_keyboard"),
        Node(
            package='go2_teleop',
            executable='teleop_keyboard',
            output='screen',
            prefix='stdbuf -o L',  # Line buffered output
        ),
    ])
