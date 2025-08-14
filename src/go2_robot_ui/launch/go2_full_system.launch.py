#!/usr/bin/env python3
"""
Full System Launch File for Go2 Robot with UI

This launch file starts both the Go2 robot SDK and the UI application:
- go2_robot_sdk for robot communication and control
- go2_robot_ui for user interface and monitoring

Environment variables required:
- ROBOT_IP: IP address of the Go2 robot
- CONN_TYPE: Connection type (webrtc or cyclonedds)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Generate launch description for full Go2 system with UI."""
    
    # Get environment variables
    robot_ip = os.getenv('ROBOT_IP', '')
    conn_type = os.getenv('CONN_TYPE', 'webrtc')
    
    # Declare launch arguments
    declare_args = [
        DeclareLaunchArgument(
            'robot_ip',
            default_value=robot_ip,
            description='IP address of the Go2 robot'
        ),
        DeclareLaunchArgument(
            'conn_type',
            default_value=conn_type,
            description='Connection type: webrtc or cyclonedx'
        ),
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='false',
            description='Whether to launch RViz (disable for UI-only setup)'
        ),
        DeclareLaunchArgument(
            'enable_nav2',
            default_value='false', 
            description='Whether to launch Nav2 navigation stack'
        ),
        DeclareLaunchArgument(
            'enable_slam',
            default_value='false',
            description='Whether to launch SLAM toolbox'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level (debug, info, warn, error)'
        ),
    ]
    
    # Include go2_robot_sdk launch file
    go2_sdk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('go2_robot_sdk'),
                'launch',
                'robot.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
            'conn_type': LaunchConfiguration('conn_type'),
            'rviz2': LaunchConfiguration('enable_rviz'),
            'nav2': LaunchConfiguration('enable_nav2'),
            'slam': LaunchConfiguration('enable_slam'),
            'joystick': 'false',  # Disable joystick when using UI
        }.items()
    )
    
    # UI Node with delay to allow robot SDK to start
    ui_node = Node(
        package='go2_robot_ui',
        executable='go2_robot_ui',
        name='go2_robot_ui_node',
        output='screen',
        parameters=[{
            'camera_topic': '/camera/image_raw',
            'webrtc_topic': '/webrtc_req',
            'robot_namespace': '',
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        # Set environment for Qt
        env={'QT_QPA_PLATFORM': 'xcb'},
        # Add delay to let robot SDK start first
        prefix='bash -c "sleep 3; exec"'
    )
    
    return LaunchDescription(declare_args + [
        go2_sdk_launch,
        ui_node
    ])


if __name__ == '__main__':
    generate_launch_description()