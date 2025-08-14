#!/usr/bin/env python3
"""
Launch file for Go2 Robot UI

This launch file starts the Go2 Robot UI application which provides:
- Camera video display with QR code detection
- Robot command interface with predefined actions  
- Robot status monitoring and feedback
- Real-time performance metrics

The UI connects to an existing go2_robot_sdk instance via ROS topics.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Go2 Robot UI."""
    
    # Declare launch arguments
    declare_args = [
        DeclareLaunchArgument(
            'camera_topic',
            default_value='/camera/image_raw',
            description='Camera topic to subscribe to (default: /camera/image_raw for single robot)'
        ),
        DeclareLaunchArgument(
            'webrtc_topic', 
            default_value='/webrtc_req',
            description='WebRTC request topic for robot commands'
        ),
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='',
            description='Robot namespace for multi-robot setups (e.g., robot0)'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level (debug, info, warn, error)'
        ),
    ]
    
    # UI Node
    ui_node = Node(
        package='go2_robot_ui',
        executable='go2_robot_ui',
        name='go2_robot_ui_node',
        output='screen',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'webrtc_topic': LaunchConfiguration('webrtc_topic'),
            'robot_namespace': LaunchConfiguration('robot_namespace'),
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        # Set environment for Qt
        env={'QT_QPA_PLATFORM': 'xcb'}  # Ensure X11 display
    )
    
    return LaunchDescription(declare_args + [ui_node])


if __name__ == '__main__':
    generate_launch_description()