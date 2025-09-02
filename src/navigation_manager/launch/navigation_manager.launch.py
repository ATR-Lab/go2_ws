#!/usr/bin/env python3
"""
Launch file for Navigation Manager

This launch file starts the navigation coordinator that integrates human detection
with Nav2 navigation for safe human-robot interaction in the petting zoo.

Author: Robot Dog Petting Zoo Team
License: BSD-3-Clause
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for navigation manager"""
    
    # Package directories
    pkg_navigation_manager = get_package_share_directory('navigation_manager')
    
    # Configuration files
    navigation_params_file = os.path.join(
        pkg_navigation_manager, 'config', 'navigation_params.yaml'
    )
    
    # Launch arguments
    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=navigation_params_file,
        description='Full path to the navigation manager parameters file'
    )
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for navigation coordinator'
    )
    
    # Navigation coordinator node
    navigation_coordinator_node = Node(
        package='navigation_manager',
        executable='navigation_coordinator',
        name='navigation_coordinator',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            # Human interaction topics (from human_interaction package)
            ('/interaction/nav_commands', '/interaction/nav_commands'),
            ('/interaction/events', '/interaction/events'),
            ('/human_detection/people', '/human_detection/people'),
            
            # Nav2 action servers (should match robot.launch.py configuration)
            ('navigate_to_pose', '/navigate_to_pose'),
            ('backup', '/backup'),
            
            # External goal interface (iPad, patrol system)
            ('/goal_pose', '/goal_pose'),
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        declare_params_file_arg,
        declare_use_sim_time_arg,
        declare_log_level_arg,
        
        # Nodes
        navigation_coordinator_node,
    ])