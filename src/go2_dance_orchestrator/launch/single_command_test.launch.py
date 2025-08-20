# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: Apache-2.0

"""
Launch file for testing single dance command execution.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for single command testing"""
    
    # Declare launch arguments
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level for dance orchestrator'
    )
    
    # Dance orchestrator node
    dance_orchestrator_node = Node(
        package='go2_dance_orchestrator',
        executable='dance_orchestrator',
        name='dance_orchestrator',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
    )
    
    return LaunchDescription([
        log_level_arg,
        dance_orchestrator_node,
    ])