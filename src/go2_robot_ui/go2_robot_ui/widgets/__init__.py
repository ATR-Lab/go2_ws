"""
Widgets Module for Go2 Robot UI

This module provides the Qt widgets for the Go2 robot user interface,
including video display, command controls, and status monitoring.

Components:
- CameraDisplay: Video display widget with QR detection overlay
- CommandPanel: Robot action control buttons and interface
- StatusPanel: Robot status and sensor data display
"""

from .camera_display import CameraDisplay
from .command_panel import CommandPanel
from .status_panel import StatusPanel

__all__ = ['CameraDisplay', 'CommandPanel', 'StatusPanel']