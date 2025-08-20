#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='go2_robot_sdk',
            executable='go2_action_service',
            name='go2_action_service',
            output='screen',
            parameters=[
                {'use_sim_time': False}
            ]
        )
    ])
