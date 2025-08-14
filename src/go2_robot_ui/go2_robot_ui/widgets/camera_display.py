"""
Camera Display Widget

This module provides a Qt widget for displaying video frames from the Go2 robot camera
with integrated QR detection overlay, performance metrics and connection status visualization.

Features:
- Video frame display with automatic scaling
- QR code detection overlay with highlighting
- Performance metrics overlay (FPS, latency)
- Connection status indication
- Frame timeout visualization
- Optimized rendering for real-time video
"""

import cv2
import numpy as np
from typing import Optional
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QLabel, QHBoxLayout
from python_qt_binding.QtGui import QImage, QPixmap, QFont, QPainter, QPen, QBrush
from python_qt_binding.QtCore import Qt, pyqtSlot
from geometry_msgs.msg import Point


class CameraDisplay(QWidget):
    """
    Widget for displaying Go2 robot camera video frames with QR detection overlay.
    
    This widget provides an optimized display for real-time video streaming
    with integrated QR detection visualization and status information.
    
    Features:
    - Automatic frame scaling to fit widget size
    - QR code detection overlay with highlighting
    - Performance metrics display (FPS, latency)
    - Connection status visualization
    - Error state handling
    """
    
    def __init__(self, parent=None):
        """
        Initialize camera display widget.
        
        Args:
            parent: Parent Qt widget (optional)
        """
        super().__init__(parent)
        
        # Display state
        self.current_frame = None
        self.current_qr_data = ""
        self.qr_center = None
        self.is_connected = False
        self.performance_stats = {}
        self.qr_highlight_time = 0
        self.qr_highlight_duration = 2.0  # seconds
        
        self._setup_ui()
        self._setup_styling()
    
    def _setup_ui(self):
        """Set up the user interface layout."""
        layout = QVBoxLayout(self)
        
        # Main video display label
        self.video_label = QLabel("Waiting for Go2 camera feed...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(640, 480)
        self.video_label.setScaledContents(False)
        layout.addWidget(self.video_label, 1)  # Give most space to video
        
        # Bottom info panel
        info_layout = QHBoxLayout()
        
        # Performance info label
        self.info_label = QLabel("Waiting for video stream...")
        self.info_label.setAlignment(Qt.AlignLeft)
        info_layout.addWidget(self.info_label, 1)
        
        # QR detection status
        self.qr_label = QLabel("QR: None")
        self.qr_label.setAlignment(Qt.AlignCenter)
        self.qr_label.setMaximumWidth(150)
        info_layout.addWidget(self.qr_label)
        
        # Connection status label
        self.status_label = QLabel("Status: Disconnected")
        self.status_label.setAlignment(Qt.AlignRight)
        self.status_label.setMaximumWidth(150)
        info_layout.addWidget(self.status_label)
        
        layout.addLayout(info_layout)
    
    def _setup_styling(self):
        """Set up widget styling."""
        # Video display styling
        self.video_label.setStyleSheet(
            "QLabel { "
            "border: 2px solid #ccc; "
            "background-color: #000; "
            "color: #fff; "
            "font-size: 14px; "
            "}"
        )
        
        # Info panel styling
        info_style = (
            "QLabel { "
            "background-color: #2b2b2b; "
            "color: #fff; "
            "padding: 5px; "
            "border: 1px solid #555; "
            "font-size: 12px; "
            "}"
        )
        
        self.info_label.setStyleSheet(info_style)
        self.qr_label.setStyleSheet(info_style + "font-weight: bold;")
        self.status_label.setStyleSheet(info_style)
        
        # Set overall widget background
        self.setStyleSheet(
            "CameraDisplay { "
            "background-color: #1e1e1e; "
            "}"
        )
    
    @pyqtSlot(np.ndarray)
    def update_frame(self, frame: np.ndarray) -> None:
        """
        Update the displayed video frame.
        
        Args:
            frame: OpenCV frame (BGR format)
        """
        try:
            if frame is None or frame.size == 0:
                return
            
            self.current_frame = frame.copy()
            
            # Convert BGR to RGB for Qt
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Create QImage
            height, width, channel = rgb_frame.shape
            bytes_per_line = 3 * width
            
            q_image = QImage(
                rgb_frame.data, 
                width, 
                height, 
                bytes_per_line, 
                QImage.Format_RGB888
            )
            
            # Create pixmap and scale to fit label while maintaining aspect ratio
            pixmap = QPixmap.fromImage(q_image)
            
            # Scale pixmap to fit the label size while maintaining aspect ratio
            label_size = self.video_label.size()
            scaled_pixmap = pixmap.scaled(
                label_size, 
                Qt.KeepAspectRatio, 
                Qt.SmoothTransformation
            )
            
            self.video_label.setPixmap(scaled_pixmap)
            
        except Exception as e:
            print(f"Error updating camera frame: {e}")
    
    @pyqtSlot(str, Point)
    def highlight_qr(self, qr_data: str, center: Point) -> None:
        """
        Highlight detected QR code.
        
        Args:
            qr_data: QR code content
            center: Center point of detected QR code
        """
        import time
        
        self.current_qr_data = qr_data
        self.qr_center = center
        self.qr_highlight_time = time.time()
        
        # Update QR status label
        display_text = qr_data if len(qr_data) <= 15 else qr_data[:12] + "..."
        self.qr_label.setText(f"QR: {display_text}")
        self.qr_label.setStyleSheet(
            "QLabel { "
            "background-color: #2d5016; "  # Green background for active QR
            "color: #fff; "
            "padding: 5px; "
            "border: 1px solid #4a7c19; "
            "font-size: 12px; "
            "font-weight: bold; "
            "}"
        )
        
        # Clear highlight after duration
        from python_qt_binding.QtCore import QTimer
        QTimer.singleShot(int(self.qr_highlight_duration * 1000), self._clear_qr_highlight)
    
    def _clear_qr_highlight(self) -> None:
        """Clear QR code highlight."""
        self.current_qr_data = ""
        self.qr_center = None
        self.qr_label.setText("QR: None")
        self.qr_label.setStyleSheet(
            "QLabel { "
            "background-color: #2b2b2b; "
            "color: #fff; "
            "padding: 5px; "
            "border: 1px solid #555; "
            "font-size: 12px; "
            "font-weight: bold; "
            "}"
        )
    
    @pyqtSlot(dict)
    def update_performance(self, metrics: dict) -> None:
        """
        Update performance metrics display.
        
        Args:
            metrics: Dictionary containing performance data
        """
        self.performance_stats = metrics
        
        # Format performance info
        fps = metrics.get('fps', 0.0)
        latency = metrics.get('latency_ms', 0.0)
        connected = metrics.get('connected', False)
        qr_enabled = metrics.get('qr_detection', False)
        
        info_text = f"FPS: {fps:.1f} | Latency: {latency:.1f}ms"
        if qr_enabled:
            info_text += " | QR: ON"
        else:
            info_text += " | QR: OFF"
        
        self.info_label.setText(info_text)
        
        # Update connection status
        if connected != self.is_connected:
            self.is_connected = connected
            self._update_connection_status(connected)
    
    @pyqtSlot()
    def on_connection_lost(self) -> None:
        """Handle camera connection lost."""
        self.is_connected = False
        self._update_connection_status(False)
        
        # Show disconnected message
        self.video_label.setPixmap(QPixmap())
        self.video_label.setText("Camera connection lost\nRetrying...")
    
    @pyqtSlot()
    def on_connection_restored(self) -> None:
        """Handle camera connection restored."""
        self.is_connected = True
        self._update_connection_status(True)
        
        # Clear disconnected message
        self.video_label.setText("")
    
    def _update_connection_status(self, connected: bool) -> None:
        """Update connection status display."""
        if connected:
            self.status_label.setText("Status: Connected")
            self.status_label.setStyleSheet(
                "QLabel { "
                "background-color: #2d5016; "  # Green for connected
                "color: #fff; "
                "padding: 5px; "
                "border: 1px solid #4a7c19; "
                "font-size: 12px; "
                "}"
            )
        else:
            self.status_label.setText("Status: Disconnected")
            self.status_label.setStyleSheet(
                "QLabel { "
                "background-color: #5c1616; "  # Red for disconnected
                "color: #fff; "
                "padding: 5px; "
                "border: 1px solid #8b2635; "
                "font-size: 12px; "
                "}"
            )
    
    def get_current_frame(self) -> Optional[np.ndarray]:
        """Get the current displayed frame."""
        return self.current_frame.copy() if self.current_frame is not None else None
    
    def get_qr_status(self) -> tuple:
        """Get current QR detection status."""
        return (self.current_qr_data, self.qr_center)
    
    def is_camera_connected(self) -> bool:
        """Check if camera is currently connected."""
        return self.is_connected