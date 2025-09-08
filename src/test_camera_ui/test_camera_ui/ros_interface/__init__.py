"""
ROS Interface Module for Test Camera UI

This module provides ROS2 communication components for the camera UI,
including frame reception and camera control interfaces.
"""

from .frame_receiver import FrameReceiver

__all__ = ['FrameReceiver']
