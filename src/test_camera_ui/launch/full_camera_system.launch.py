#!/usr/bin/env python3
"""
Full Camera System Launch

Launches both the threaded camera node and the optimized UI for a complete
camera system with separated processing and display components.

This demonstrates the full optimized architecture:
- Threaded camera capture and publishing
- Separated Qt-based UI with performance monitoring
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Threaded Test Camera Node (processing)
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
                'enable_preview': False  # Disabled since we have separate UI
            }]
        ),
        
        # Optimized Test Camera UI (display)
        Node(
            package='test_camera_ui',
            executable='test_camera_ui_node',
            name='test_camera_ui_node',
            output='screen'
        )
    ])
