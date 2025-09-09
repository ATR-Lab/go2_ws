"""
Status Panel Widget

This module provides a Qt widget for displaying Go2 robot status information including:
- Robot state data (mode, position, velocity)
- IMU sensor data (orientation, acceleration)
- Joint states and motor information
- Connection status and diagnostics
- Command execution feedback

The status panel provides real-time monitoring of robot health and state.
"""

import math
from typing import Dict, Any, Optional
from python_qt_binding.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, 
                                       QLabel, QGroupBox, QGridLayout, 
                                       QProgressBar, QScrollArea)
from python_qt_binding.QtCore import Qt, pyqtSlot, QTimer
from python_qt_binding.QtGui import QFont


class StatusPanel(QWidget):
    """
    Widget for displaying Go2 robot status and sensor data.
    
    This widget provides comprehensive robot status monitoring including
    robot state, sensor data, connection status, and command feedback.
    
    Features:
    - Real-time robot state display
    - IMU sensor visualization
    - Joint state monitoring
    - Connection status indication
    - Command execution feedback
    - Performance metrics
    """
    
    def __init__(self, parent=None):
        """
        Initialize status panel widget.
        
        Args:
            parent: Parent Qt widget (optional)
        """
        super().__init__(parent)
        
        # Status data
        self.robot_state = {}
        self.imu_data = {}
        self.joint_data = {}
        self.is_connected = False
        
        # UI components
        self.status_labels = {}
        self.progress_bars = {}
        
        self._setup_ui()
        
        # Update timer for calculated values
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self._update_calculated_values)
        self.update_timer.start(100)  # Update every 100ms
    
    def _setup_ui(self):
        """Set up the user interface layout."""
        # Main layout with scroll area
        main_layout = QVBoxLayout(self)
        
        # Title
        title_label = QLabel("Robot Status")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet(
            "QLabel { "
            "font-size: 16px; "
            "font-weight: bold; "
            "color: #fff; "
            "padding: 10px; "
            "background-color: #2b2b2b; "
            "border: 1px solid #555; "
            "border-radius: 5px; "
            "}"
        )
        main_layout.addWidget(title_label)
        
        # Scroll area for status info
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        
        # Status widget
        status_widget = QWidget()
        status_layout = QVBoxLayout(status_widget)
        
        # Create status groups
        status_layout.addWidget(self._create_connection_group())
        status_layout.addWidget(self._create_robot_state_group())
        status_layout.addWidget(self._create_imu_group())
        status_layout.addWidget(self._create_joint_group())
        status_layout.addWidget(self._create_command_feedback_group())
        
        # Add stretch
        status_layout.addStretch()
        
        scroll_area.setWidget(status_widget)
        main_layout.addWidget(scroll_area)
        
        # Set overall styling
        self.setStyleSheet(
            "StatusPanel { "
            "background-color: #1e1e1e; "
            "} "
            "QScrollArea { "
            "border: none; "
            "background-color: #1e1e1e; "
            "} "
            "QScrollBar:vertical { "
            "background-color: #2b2b2b; "
            "width: 12px; "
            "border: none; "
            "} "
            "QScrollBar::handle:vertical { "
            "background-color: #555; "
            "border-radius: 6px; "
            "} "
            "QScrollBar::handle:vertical:hover { "
            "background-color: #777; "
            "}"
        )
    
    def _create_connection_group(self) -> QGroupBox:
        """Create connection status group."""
        group = QGroupBox("Connection")
        group.setStyleSheet(self._get_group_style())
        
        layout = QVBoxLayout(group)
        
        # Connection status
        self.status_labels['connection'] = QLabel("Disconnected")
        self.status_labels['connection'].setStyleSheet(self._get_status_label_style())
        layout.addWidget(self.status_labels['connection'])
        
        # Data rates
        self.status_labels['data_rates'] = QLabel("Robot State: -- | IMU: -- | Joints: --")
        self.status_labels['data_rates'].setStyleSheet(self._get_info_label_style())
        layout.addWidget(self.status_labels['data_rates'])
        
        return group
    
    def _create_robot_state_group(self) -> QGroupBox:
        """Create robot state information group."""
        group = QGroupBox("Robot State")
        group.setStyleSheet(self._get_group_style())
        
        layout = QGridLayout(group)
        
        # Mode and gait
        layout.addWidget(QLabel("Mode:"), 0, 0)
        self.status_labels['mode'] = QLabel("--")
        self.status_labels['mode'].setStyleSheet(self._get_value_label_style())
        layout.addWidget(self.status_labels['mode'], 0, 1)
        
        layout.addWidget(QLabel("Gait:"), 1, 0)
        self.status_labels['gait'] = QLabel("--")
        self.status_labels['gait'].setStyleSheet(self._get_value_label_style())
        layout.addWidget(self.status_labels['gait'], 1, 1)
        
        # Position
        layout.addWidget(QLabel("Position (m):"), 2, 0, 1, 2)
        self.status_labels['position'] = QLabel("X: -- Y: -- Z: --")
        self.status_labels['position'].setStyleSheet(self._get_value_label_style())
        layout.addWidget(self.status_labels['position'], 3, 0, 1, 2)
        
        # Velocity
        layout.addWidget(QLabel("Velocity (m/s):"), 4, 0, 1, 2)
        self.status_labels['velocity'] = QLabel("X: -- Y: -- Z: --")
        self.status_labels['velocity'].setStyleSheet(self._get_value_label_style())
        layout.addWidget(self.status_labels['velocity'], 5, 0, 1, 2)
        
        # Body height
        layout.addWidget(QLabel("Body Height:"), 6, 0)
        self.status_labels['body_height'] = QLabel("--")
        self.status_labels['body_height'].setStyleSheet(self._get_value_label_style())
        layout.addWidget(self.status_labels['body_height'], 6, 1)
        
        return group
    
    def _create_imu_group(self) -> QGroupBox:
        """Create IMU sensor data group."""
        group = QGroupBox("IMU Sensor")
        group.setStyleSheet(self._get_group_style())
        
        layout = QGridLayout(group)
        
        # Orientation (roll, pitch, yaw) - using direct RPY values
        layout.addWidget(QLabel("Orientation (°):"), 0, 0, 1, 2)
        self.status_labels['orientation'] = QLabel("R: -- P: -- Y: --")
        self.status_labels['orientation'].setStyleSheet(self._get_value_label_style())
        layout.addWidget(self.status_labels['orientation'], 1, 0, 1, 2)
        
        # Quaternion
        layout.addWidget(QLabel("Quaternion:"), 2, 0, 1, 2)
        self.status_labels['quaternion'] = QLabel("X: -- Y: -- Z: -- W: --")
        self.status_labels['quaternion'].setStyleSheet(self._get_value_label_style())
        layout.addWidget(self.status_labels['quaternion'], 3, 0, 1, 2)
        
        # Gyroscope (angular velocity)
        layout.addWidget(QLabel("Gyroscope (°/s):"), 4, 0, 1, 2)
        self.status_labels['gyroscope'] = QLabel("X: -- Y: -- Z: --")
        self.status_labels['gyroscope'].setStyleSheet(self._get_value_label_style())
        layout.addWidget(self.status_labels['gyroscope'], 5, 0, 1, 2)
        
        # Accelerometer
        layout.addWidget(QLabel("Accelerometer (m/s²):"), 6, 0, 1, 2)
        self.status_labels['accelerometer'] = QLabel("X: -- Y: -- Z: --")
        self.status_labels['accelerometer'].setStyleSheet(self._get_value_label_style())
        layout.addWidget(self.status_labels['accelerometer'], 7, 0, 1, 2)
        
        # Temperature
        layout.addWidget(QLabel("Temperature:"), 8, 0)
        self.status_labels['temperature'] = QLabel("--°C")
        self.status_labels['temperature'].setStyleSheet(self._get_value_label_style())
        layout.addWidget(self.status_labels['temperature'], 8, 1)
        
        return group
    
    def _create_joint_group(self) -> QGroupBox:
        """Create joint states group."""
        group = QGroupBox("Joint States")
        group.setStyleSheet(self._get_group_style())
        
        layout = QVBoxLayout(group)
        
        # Joint count
        self.status_labels['joint_count'] = QLabel("Joints: --")
        self.status_labels['joint_count'].setStyleSheet(self._get_value_label_style())
        layout.addWidget(self.status_labels['joint_count'])
        
        # Joint summary
        self.status_labels['joint_summary'] = QLabel("Position range: -- | Velocity range: --")
        self.status_labels['joint_summary'].setStyleSheet(self._get_value_label_style())
        layout.addWidget(self.status_labels['joint_summary'])
        
        return group
    
    def _create_command_feedback_group(self) -> QGroupBox:
        """Create command feedback group."""
        group = QGroupBox("Command Status")
        group.setStyleSheet(self._get_group_style())
        
        layout = QVBoxLayout(group)
        
        # Last command
        self.status_labels['last_command'] = QLabel("No commands sent")
        self.status_labels['last_command'].setStyleSheet(self._get_info_label_style())
        layout.addWidget(self.status_labels['last_command'])
        
        # Command status
        self.status_labels['command_status'] = QLabel("Ready")
        self.status_labels['command_status'].setStyleSheet(self._get_status_label_style())
        layout.addWidget(self.status_labels['command_status'])
        
        return group
    
    def _get_group_style(self) -> str:
        """Get standard group box styling."""
        return (
            "QGroupBox { "
            "font-size: 12px; "
            "font-weight: bold; "
            "color: #fff; "
            "border: 2px solid #555; "
            "border-radius: 5px; "
            "margin-top: 10px; "
            "padding-top: 10px; "
            "} "
            "QGroupBox::title { "
            "subcontrol-origin: margin; "
            "left: 10px; "
            "padding: 0 5px 0 5px; "
            "}"
        )
    
    def _get_status_label_style(self) -> str:
        """Get styling for status labels."""
        return (
            "QLabel { "
            "color: #fff; "
            "font-size: 11px; "
            "font-weight: bold; "
            "padding: 2px; "
            "border: 1px solid #555; "
            "border-radius: 3px; "
            "background-color: #2b2b2b; "
            "}"
        )
    
    def _get_value_label_style(self) -> str:
        """Get styling for value labels."""
        return (
            "QLabel { "
            "color: #4ade80; "
            "font-size: 11px; "
            "font-weight: bold; "
            "padding: 2px; "
            "}"
        )
    
    def _get_info_label_style(self) -> str:
        """Get styling for info labels."""
        return (
            "QLabel { "
            "color: #e5e7eb; "
            "font-size: 10px; "
            "padding: 2px; "
            "}"
        )
    
    @pyqtSlot(dict)
    def update_robot_state(self, state_data: Dict[str, Any]) -> None:
        """Update robot state display."""
        self.robot_state = state_data
        
        try:
            # Mode
            mode = state_data.get('mode', 0)
            mode_names = {0: "Idle", 1: "Sport", 2: "Walk", 3: "Stand", 4: "Damping"}
            self.status_labels['mode'].setText(mode_names.get(mode, f"Mode {mode}"))
            
            # Gait type
            gait = state_data.get('gait_type', 0)
            gait_names = {0: "Idle", 1: "Trot", 2: "Walk", 3: "Bound"}
            self.status_labels['gait'].setText(gait_names.get(gait, f"Gait {gait}"))
            
            # Position
            pos = state_data.get('position', [0, 0, 0])
            if len(pos) >= 3:
                self.status_labels['position'].setText(f"X: {pos[0]:.2f} Y: {pos[1]:.2f} Z: {pos[2]:.2f}")
            
            # Velocity
            vel = state_data.get('velocity', [0, 0, 0])
            if len(vel) >= 3:
                self.status_labels['velocity'].setText(f"X: {vel[0]:.2f} Y: {vel[1]:.2f} Z: {vel[2]:.2f}")
            
            # Body height
            height = state_data.get('body_height', 0.0)
            self.status_labels['body_height'].setText(f"{height:.3f} m")
            
        except Exception as e:
            print(f"Error updating robot state display: {e}")
    
    @pyqtSlot(dict)
    def update_imu(self, imu_data: Dict[str, Any]) -> None:
        """Update IMU data display."""
        self.imu_data = imu_data
        
        try:
            # Orientation using direct RPY values
            rpy = imu_data.get('rpy', {})
            if rpy:
                self.status_labels['orientation'].setText(
                    f"R: {math.degrees(rpy.get('roll', 0)):.1f} "
                    f"P: {math.degrees(rpy.get('pitch', 0)):.1f} "
                    f"Y: {math.degrees(rpy.get('yaw', 0)):.1f}"
                )
            
            # Quaternion
            quaternion = imu_data.get('quaternion', {})
            if quaternion:
                self.status_labels['quaternion'].setText(
                    f"X: {quaternion.get('x', 0):.3f} "
                    f"Y: {quaternion.get('y', 0):.3f} "
                    f"Z: {quaternion.get('z', 0):.3f} "
                    f"W: {quaternion.get('w', 0):.3f}"
                )
            
            # Gyroscope (angular velocity)
            gyroscope = imu_data.get('gyroscope', {})
            if gyroscope:
                self.status_labels['gyroscope'].setText(
                    f"X: {math.degrees(gyroscope.get('x', 0)):.1f} "
                    f"Y: {math.degrees(gyroscope.get('y', 0)):.1f} "
                    f"Z: {math.degrees(gyroscope.get('z', 0)):.1f}"
                )
            
            # Accelerometer
            accelerometer = imu_data.get('accelerometer', {})
            if accelerometer:
                self.status_labels['accelerometer'].setText(
                    f"X: {accelerometer.get('x', 0):.2f} "
                    f"Y: {accelerometer.get('y', 0):.2f} "
                    f"Z: {accelerometer.get('z', 0):.2f}"
                )
            
            # Temperature
            temperature = imu_data.get('temperature', 0)
            self.status_labels['temperature'].setText(f"{temperature}°C")
            
        except Exception as e:
            print(f"Error updating IMU display: {e}")
            import traceback
            traceback.print_exc()
    
    @pyqtSlot(dict)
    def update_joints(self, joint_data: Dict[str, Any]) -> None:
        """Update joint states display."""
        self.joint_data = joint_data
        
        try:
            names = joint_data.get('names', [])
            positions = joint_data.get('positions', [])
            velocities = joint_data.get('velocities', [])
            
            # Joint count
            self.status_labels['joint_count'].setText(f"Joints: {len(names)}")
            
            # Position and velocity ranges
            if positions:
                pos_min, pos_max = min(positions), max(positions)
                pos_range = f"{pos_min:.2f} to {pos_max:.2f}"
            else:
                pos_range = "--"
            
            if velocities:
                vel_min, vel_max = min(velocities), max(velocities)
                vel_range = f"{vel_min:.2f} to {vel_max:.2f}"
            else:
                vel_range = "--"
            
            self.status_labels['joint_summary'].setText(
                f"Position range: {pos_range} | Velocity range: {vel_range}"
            )
            
        except Exception as e:
            print(f"Error updating joint display: {e}")
    
    @pyqtSlot(bool)
    def update_connection_status(self, connected: bool) -> None:
        """Update connection status display."""
        self.is_connected = connected
        
        if connected:
            self.status_labels['connection'].setText("Connected")
            self.status_labels['connection'].setStyleSheet(
                self._get_status_label_style() + "background-color: #2d5016;"
            )
        else:
            self.status_labels['connection'].setText("Disconnected")
            self.status_labels['connection'].setStyleSheet(
                self._get_status_label_style() + "background-color: #5c1616;"
            )
    
    @pyqtSlot(str)
    def show_command_feedback(self, command: str) -> None:
        """Show command execution feedback."""
        self.status_labels['last_command'].setText(f"Executed: {command}")
        self.status_labels['command_status'].setText("Command Sent")
        self.status_labels['command_status'].setStyleSheet(
            self._get_status_label_style() + "background-color: #2d5016;"
        )
        
        # Reset status after 3 seconds
        QTimer.singleShot(3000, self._reset_command_status)
    
    @pyqtSlot(str)
    def show_error_message(self, error: str) -> None:
        """Show command error feedback."""
        self.status_labels['command_status'].setText(f"Error: {error}")
        self.status_labels['command_status'].setStyleSheet(
            self._get_status_label_style() + "background-color: #5c1616;"
        )
        
        # Reset status after 5 seconds
        QTimer.singleShot(5000, self._reset_command_status)
    
    def _reset_command_status(self) -> None:
        """Reset command status to ready."""
        self.status_labels['command_status'].setText("Ready")
        self.status_labels['command_status'].setStyleSheet(self._get_status_label_style())
    
    def _update_calculated_values(self) -> None:
        """Update calculated values and derived displays."""
        # This can be used for real-time calculations or animations
        pass
    
    def _quaternion_to_euler(self, x: float, y: float, z: float, w: float) -> tuple:
        """
        Convert quaternion to euler angles (roll, pitch, yaw).
        
        Returns:
            Tuple of (roll, pitch, yaw) in radians
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw