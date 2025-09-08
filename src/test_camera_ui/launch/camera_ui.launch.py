#!/usr/bin/env python3
"""
Qt Camera UI Only Launch

Launches ONLY the optimized Qt-based camera UI. Expects a camera node
to already be running and publishing to /camera/image_raw topic.

Usage:
  Terminal 1: ros2 launch test_camera threaded_camera_only.launch.py
  Terminal 2: ros2 launch test_camera_ui camera_ui.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Qt Camera UI Node (expects camera from elsewhere)
        Node(
            package='test_camera_ui',
            executable='test_camera_ui_node',
            name='test_camera_ui_node',
            output='screen',
            parameters=[{
                # UI can be configured here if needed
            }]
        )
    ])
