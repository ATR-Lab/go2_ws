#!/usr/bin/env python3
"""
Launch file for Sui QR Processor Node

This launch file starts the sui_qr_processor_node with configurable parameters.
It supports different network configurations and camera topics.

Usage:
    ros2 launch sui_qr_processor sui_qr_processor.launch.py
    ros2 launch sui_qr_processor sui_qr_processor.launch.py network:=mainnet
    ros2 launch sui_qr_processor sui_qr_processor.launch.py camera_topic:=/my_camera/image_raw
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for sui_qr_processor."""
    
    # Get package directory
    pkg_share = FindPackageShare('sui_qr_processor')
    
    # Default config file path
    default_config_file = PathJoinSubstitution([
        pkg_share, 'config', 'sui_qr_processor.yaml'
    ])
    
    # Declare launch arguments
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to configuration file'
    )
    
    declare_camera_topic = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Camera image topic to subscribe to'
    )
    
    declare_network = DeclareLaunchArgument(
        'network',
        default_value='testnet',
        description='Default Sui network (testnet, mainnet, devnet)'
    )
    
    declare_network_source = DeclareLaunchArgument(
        'network_source',
        default_value='qr_with_fallback',
        description='Network selection strategy (qr_with_fallback, config_only, qr_only)'
    )
    
    declare_execution_cooldown = DeclareLaunchArgument(
        'execution_cooldown',
        default_value='30.0',
        description='Minimum seconds between transaction executions'
    )
    
    declare_qr_detection_interval = DeclareLaunchArgument(
        'qr_detection_interval',
        default_value='0.2',
        description='QR detection processing interval in seconds'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error)'
    )
    
    # Create the node
    sui_qr_processor_node = Node(
        package='sui_qr_processor',
        executable='sui_qr_processor_node',
        name='sui_qr_processor_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'camera_topic': LaunchConfiguration('camera_topic'),
                'default_network': LaunchConfiguration('network'),
                'network_source': LaunchConfiguration('network_source'),
                'execution_cooldown': LaunchConfiguration('execution_cooldown'),
                'qr_detection_interval': LaunchConfiguration('qr_detection_interval'),
            }
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            # Remap topics if needed
            # ('~/camera/image_raw', LaunchConfiguration('camera_topic')),
        ]
    )
    
    return LaunchDescription([
        declare_config_file,
        declare_camera_topic,
        declare_network,
        declare_network_source,
        declare_execution_cooldown,
        declare_qr_detection_interval,
        declare_log_level,
        sui_qr_processor_node,
    ])


if __name__ == '__main__':
    generate_launch_description()
