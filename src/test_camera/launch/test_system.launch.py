#!/usr/bin/env python3
"""
Test System Launch File

Launches the complete test system for the robot dog petting zoo:
- test_camera_node: PC camera feed
- human_detection_node: YOLO + MediaPipe detection
- interaction_manager_node: Interaction state machine
- detection_visualizer: PyQt5 visualization UI
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    test_camera_dir = get_package_share_directory('test_camera')
    human_interaction_dir = get_package_share_directory('human_interaction')
    
    # Launch arguments
    enable_camera = LaunchConfiguration('enable_camera', default='true')
    enable_detection = LaunchConfiguration('enable_detection', default='true')
    enable_interaction = LaunchConfiguration('enable_interaction', default='true')
    enable_visualizer = LaunchConfiguration('enable_visualizer', default='true')
    
    # Camera parameters
    camera_index = LaunchConfiguration('camera_index', default='0')
    frame_rate = LaunchConfiguration('frame_rate', default='30.0')
    image_width = LaunchConfiguration('image_width', default='640')
    image_height = LaunchConfiguration('image_height', default='480')
    
    # Detection parameters
    detection_confidence = LaunchConfiguration('detection_confidence', default='0.6')
    processing_frequency = LaunchConfiguration('processing_frequency', default='10.0')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'enable_camera',
            default_value='true',
            description='Enable test camera node'
        ),
        
        DeclareLaunchArgument(
            'enable_detection',
            default_value='true',
            description='Enable human detection node'
        ),
        
        DeclareLaunchArgument(
            'enable_interaction',
            default_value='true',
            description='Enable interaction manager node'
        ),
        
        DeclareLaunchArgument(
            'enable_visualizer',
            default_value='true',
            description='Enable PyQt5 visualization UI'
        ),
        
        DeclareLaunchArgument(
            'camera_index',
            default_value='0',
            description='Camera device index'
        ),
        
        DeclareLaunchArgument(
            'frame_rate',
            default_value='30.0',
            description='Camera frame rate (Hz)'
        ),
        
        DeclareLaunchArgument(
            'image_width',
            default_value='640',
            description='Image width (pixels)'
        ),
        
        DeclareLaunchArgument(
            'image_height',
            default_value='480',
            description='Image height (pixels)'
        ),
        
        DeclareLaunchArgument(
            'detection_confidence',
            default_value='0.6',
            description='YOLO detection confidence threshold'
        ),
        
        DeclareLaunchArgument(
            'processing_frequency',
            default_value='10.0',
            description='Detection processing frequency (Hz)'
        ),
        
        # Test Camera Node
        Node(
            package='test_camera',
            executable='test_camera_node',
            name='test_camera_node',
            output='screen',
            condition=IfCondition(enable_camera),
            parameters=[{
                'camera_index': camera_index,
                'frame_rate': frame_rate,
                'image_width': image_width,
                'image_height': image_height,
                'enable_preview': False  # Disable OpenCV preview since we have PyQt5 UI
            }]
        ),
        
        # Human Detection Node
        Node(
            package='human_interaction',
            executable='human_detection_node',
            name='human_detection_node',
            output='screen',
            condition=IfCondition(enable_detection),
            parameters=[{
                'detection_confidence': detection_confidence,
                'processing_frequency': processing_frequency,
                'max_detection_distance': 10.0,
                'interaction_zone_distance': 2.0,
                'approach_zone_distance': 5.0,
                'enable_pose_estimation': True,
                'enable_gesture_recognition': True
            }]
        ),
        
        # Interaction Manager Node
        Node(
            package='human_interaction',
            executable='interaction_manager_node',
            name='interaction_manager_node',
            output='screen',
            condition=IfCondition(enable_interaction),
            parameters=[{
                'state_timeout': 30.0,
                'max_interaction_time': 60.0,
                'resume_patrol_delay': 5.0,
                'enable_automatic_resume': True,
                'enable_gesture_responses': True,
                'enable_speech_responses': True
            }]
        ),
        
        # Detection Visualizer UI (PyQt5)
        ExecuteProcess(
            cmd=['ros2', 'run', 'test_camera', 'detection_visualizer'],
            output='screen',
            condition=IfCondition(enable_visualizer)
        )
    ]) 