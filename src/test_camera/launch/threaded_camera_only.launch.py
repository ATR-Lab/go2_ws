#!/usr/bin/env python3
"""
Threaded Camera Only Launch

Launches ONLY the multi-threaded camera node without any display.
This is for testing camera performance in isolation or when using
a separate UI package.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Threaded Test Camera Node (no display)
        Node(
            package='test_camera',
            executable='test_camera_threaded_node',
            name='test_camera_threaded_node',
            output='screen',
            parameters=[{
                'camera_index': 0,
                'frame_rate': 30.0,
                'image_width': 640,
                'image_height': 480,
                'enable_preview': False  # No preview - headless operation
            }]
        )
    ])
