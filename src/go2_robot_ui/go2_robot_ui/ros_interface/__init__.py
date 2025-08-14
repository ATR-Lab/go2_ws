"""
ROS Interface Module for Go2 Robot UI

This module provides the ROS communication layer for the Go2 robot UI,
handling all interactions with the go2_robot_sdk and other ROS nodes.

Components:
- CameraReceiver: Handles camera video stream with QR detection
- RobotController: Sends commands to the Go2 robot
- StatusMonitor: Monitors robot state and sensor data
"""

from .camera_receiver import CameraReceiver
from .robot_controller import RobotController
from .status_monitor import StatusMonitor

__all__ = ['CameraReceiver', 'RobotController', 'StatusMonitor']