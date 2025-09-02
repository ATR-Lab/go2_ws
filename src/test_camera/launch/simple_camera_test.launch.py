#!/usr/bin/env python3
"""
Simple Camera Test Launch

Launches just the test camera and simple display for basic testing.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Test Camera Node
        Node(
            package='test_camera',
            executable='test_camera_node',
            name='test_camera_node',
            output='screen',
            parameters=[{
                'camera_index': 0,
                'frame_rate': 30.0,
                'image_width': 640,
                'image_height': 480,
                'enable_preview': False
            }]
        ),
        
        # Simple Camera Display
        Node(
            package='test_camera',
            executable='simple_camera_display',
            name='simple_camera_display',
            output='screen'
        )
    ]) 