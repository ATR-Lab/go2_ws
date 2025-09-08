#!/usr/bin/env python3
"""
Threaded Camera Only Launch

Launches ONLY the multi-threaded camera node without any display.
This is for testing camera performance in isolation or when using
a separate UI package.

Usage:
  # Use laptop camera (index 0)
  ros2 launch test_camera threaded_camera_only.launch.py camera_index:=0
  
  # Use Logitech BRIO (index 4)  
  ros2 launch test_camera threaded_camera_only.launch.py camera_index:=4
  
  # Default is Logitech BRIO
  ros2 launch test_camera threaded_camera_only.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare camera_index parameter
        DeclareLaunchArgument(
            'camera_index',
            default_value='4',  # Default to Logitech BRIO
            description='Camera index: 0=laptop camera, 4=Logitech BRIO'
        ),
        
        # Threaded Test Camera Node (no display)
        Node(
            package='test_camera',
            executable='test_camera_threaded_node',
            name='test_camera_threaded_node',
            output='screen',
            parameters=[{
                'camera_index': LaunchConfiguration('camera_index'),
                'frame_rate': 30.0,
                'image_width': 640,
                'image_height': 480,
                'enable_preview': False  # No preview - headless operation
            }]
        )
    ])
