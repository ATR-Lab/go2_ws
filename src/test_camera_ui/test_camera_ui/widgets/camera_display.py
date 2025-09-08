"""
Camera Display Widget for Test Camera UI

This module provides a Qt widget for displaying video frames from the camera
with integrated performance metrics and connection status visualization.

Features:
- Video frame display with automatic scaling
- Performance metrics overlay
- Connection status indication
- Frame timeout visualization
- Optimized rendering for real-time video
"""

import cv2
import numpy as np
from python_qt_binding.QtWidgets import QWidget, QVBoxLayout, QLabel, QHBoxLayout
from python_qt_binding.QtGui import QImage, QPixmap, QFont, QPainter, QPen, QBrush
from python_qt_binding.QtCore import Qt, pyqtSlot


class CameraDisplay(QWidget):
    """
    Widget for displaying camera video frames with performance metrics.
    
    This widget provides an optimized display for real-time video streaming
    with integrated status information and connection monitoring.
    
    Features:
    - Automatic frame scaling to fit widget size
    - Performance metrics display (FPS, latency)
    - Connection status visualization
    - Error state handling
    - Professional UI styling
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
        self.is_connected = False
        self.performance_stats = {}
        
        # Frame scaling optimization cache
        self.cached_scale_factor = None
        self.cached_label_size = (0, 0)
        self.cached_frame_size = (0, 0)
        
        self._setup_ui()
        self._setup_styling()
    
    def _setup_ui(self):
        """Set up the user interface layout."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(5)
        
        # Main video display label
        self.video_label = QLabel("Initializing Test Camera Display...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setMinimumSize(640, 480)
        self.video_label.setScaledContents(False)  # We'll handle scaling manually
        layout.addWidget(self.video_label, 1)  # Give most space to video
        
        # Performance metrics row
        metrics_layout = QHBoxLayout()
        
        # Performance info label
        self.info_label = QLabel("Waiting for camera stream...")
        self.info_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        metrics_layout.addWidget(self.info_label, 1)
        
        # Status label (right-aligned)
        self.status_label = QLabel("Status: Disconnected")
        self.status_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        metrics_layout.addWidget(self.status_label)
        
        layout.addLayout(metrics_layout)
    
    def _setup_styling(self):
        """Set up widget styling."""
        # Main widget styling
        self.setStyleSheet("""
            QWidget {
                background-color: #2b2b2b;
                color: #ffffff;
            }
        """)
        
        # Video display styling
        self.video_label.setStyleSheet("""
            QLabel {
                border: 2px solid #555555;
                background-color: #1a1a1a;
                color: #cccccc;
                font-size: 14px;
                font-weight: bold;
                border-radius: 4px;
                padding: 10px;
            }
        """)
        
        # Info label styling
        self.info_label.setStyleSheet("""
            QLabel {
                background-color: #3a3a3a;
                color: #ffffff;
                padding: 8px 12px;
                border-radius: 4px;
                font-family: 'Consolas', 'Monaco', monospace;
                font-size: 11px;
                border: 1px solid #555555;
            }
        """)
        
        # Status label styling
        self.status_label.setStyleSheet("""
            QLabel {
                color: #cccccc;
                font-size: 11px;
                font-weight: bold;
                padding: 8px 12px;
                border-radius: 4px;
                background-color: #3a3a3a;
                border: 1px solid #555555;
            }
        """)
    
    @pyqtSlot(np.ndarray, float)
    def update_frame(self, frame, latency):
        """
        Update display with new video frame.
        
        Args:
            frame: OpenCV frame (BGR format)
            latency: Frame latency in seconds
        """
        try:
            if frame is None:
                return
            
            # Store current frame
            self.current_frame = frame
            
            # Convert BGR to RGB for Qt display
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Scale frame to fit label while preserving aspect ratio
            scaled_frame = self._scale_frame_to_fit(rgb_frame)
            
            # Convert to QPixmap and display
            height, width, channel = scaled_frame.shape
            bytes_per_line = 3 * width
            q_image = QImage(scaled_frame.data, width, height, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(q_image)
            
            self.video_label.setPixmap(pixmap)
            
            # Update connection status
            if not self.is_connected:
                self.is_connected = True
                self._update_connection_status(True)
        
        except Exception as e:
            self._show_error_state(f"Frame display error: {e}")
    
    @pyqtSlot(dict)
    def update_performance(self, stats):
        """
        Update performance metrics display.
        
        Args:
            stats: Dictionary containing performance metrics
        """
        self.performance_stats = stats
        self._update_info_display()
    
    @pyqtSlot()
    def on_connection_lost(self):
        """Handle connection lost signal."""
        self.is_connected = False
        self._update_connection_status(False)
        self._show_connection_lost_state()
    
    @pyqtSlot()
    def on_connection_restored(self):
        """Handle connection restored signal."""
        self.is_connected = True
        self._update_connection_status(True)
    
    def _scale_frame_to_fit(self, frame):
        """
        Scale frame to fit within the video label while preserving aspect ratio.
        Uses caching to avoid unnecessary cv2.resize() operations.
        
        Args:
            frame: OpenCV frame to scale
            
        Returns:
            Scaled frame
        """
        label_width = self.video_label.width() - 20  # Account for padding
        label_height = self.video_label.height() - 20
        
        if label_width <= 0 or label_height <= 0:
            return frame
        
        height, width = frame.shape[:2]
        current_label_size = (label_width, label_height)
        current_frame_size = (width, height)
        
        # Check if we need to recalculate scale factor
        if (self.cached_scale_factor is None or 
            current_label_size != self.cached_label_size or 
            current_frame_size != self.cached_frame_size):
            
            # Calculate scale factor to fit in label while preserving aspect ratio
            scale_w = label_width / width
            scale_h = label_height / height
            scale = min(scale_w, scale_h, 1.0)  # Don't upscale
            
            # Cache the results
            self.cached_scale_factor = scale
            self.cached_label_size = current_label_size
            self.cached_frame_size = current_frame_size
        
        # Use cached scale factor
        scale = self.cached_scale_factor
        
        # Only resize if scaling is needed
        if scale < 1.0:
            new_size = (int(width * scale), int(height * scale))
            frame = cv2.resize(frame, new_size, interpolation=cv2.INTER_AREA)
        
        return frame
    
    def _update_info_display(self):
        """Update the performance information display."""
        if not self.performance_stats:
            return
        
        stats = self.performance_stats
        
        # Format performance metrics
        fps = stats.get('fps', 0)
        frame_count = stats.get('frame_count', 0)
        avg_latency = stats.get('avg_latency_ms', 0)
        max_latency = stats.get('max_latency_ms', 0)
        elapsed = stats.get('elapsed_time', 0)
        
        info_text = (
            f"FPS: {fps:.1f} | "
            f"Frames: {frame_count} | "
            f"Latency: {avg_latency:.1f}ms | "
            f"Max: {max_latency:.1f}ms | "
            f"Runtime: {elapsed:.1f}s"
        )
        
        if self.current_frame is not None:
            height, width = self.current_frame.shape[:2]
            info_text += f" | Resolution: {width}x{height}"
        
        self.info_label.setText(info_text)
    
    def _update_connection_status(self, connected):
        """
        Update connection status display.
        
        Args:
            connected: True if connected, False if disconnected
        """
        if connected:
            self.status_label.setText("Status: Connected")
            self.status_label.setStyleSheet("""
                QLabel {
                    color: #4caf50;
                    font-size: 11px;
                    font-weight: bold;
                    padding: 8px 12px;
                    border-radius: 4px;
                    background-color: #2e7d32;
                    border: 1px solid #4caf50;
                }
            """)
            # Update video label border to green
            self.video_label.setStyleSheet("""
                QLabel {
                    border: 2px solid #4caf50;
                    background-color: #1a1a1a;
                    color: #cccccc;
                    font-size: 14px;
                    font-weight: bold;
                    border-radius: 4px;
                    padding: 10px;
                }
            """)
        else:
            self.status_label.setText("Status: Disconnected")
            self.status_label.setStyleSheet("""
                QLabel {
                    color: #f44336;
                    font-size: 11px;
                    font-weight: bold;
                    padding: 8px 12px;
                    border-radius: 4px;
                    background-color: #d32f2f;
                    border: 1px solid #f44336;
                }
            """)
    
    def _show_connection_lost_state(self):
        """Show connection lost visual state."""
        self.video_label.setText(
            "📡 Connection Lost\n\n"
            "No video stream received from camera node\n\n"
            "• Check if test_camera_node is running\n"
            "• Verify /camera/image_raw topic is publishing\n"
            "• Check camera hardware connection"
        )
        self.video_label.setStyleSheet("""
            QLabel {
                border: 2px solid #f44336;
                background-color: #1a1a1a;
                color: #ff6666;
                font-size: 13px;
                font-weight: bold;
                border-radius: 4px;
                padding: 20px;
            }
        """)
        self.info_label.setText("Connection lost - No frames received")
    
    def _show_error_state(self, error_message):
        """
        Show error state in the display.
        
        Args:
            error_message: Error message to display
        """
        self.video_label.setText(
            f"⚠️ Display Error\n\n{error_message}\n\n"
            "Check logs for detailed error information"
        )
        self.video_label.setStyleSheet("""
            QLabel {
                border: 2px solid #ff9800;
                background-color: #1a1a1a;
                color: #ffb74d;
                font-size: 13px;
                font-weight: bold;
                border-radius: 4px;
                padding: 20px;
            }
        """)
        self.info_label.setText(f"Error: {error_message}")
    
    def resizeEvent(self, event):
        """Handle widget resize events to invalidate scaling cache."""
        super().resizeEvent(event)
        # Invalidate scaling cache when widget is resized
        self.cached_scale_factor = None
        self.cached_label_size = (0, 0)
    
    def get_current_frame_info(self):
        """
        Get information about the currently displayed frame.
        
        Returns:
            dict: Frame information (size, format, etc.)
        """
        if self.current_frame is None:
            return {}
        
        height, width, channels = self.current_frame.shape
        return {
            'width': width,
            'height': height,
            'channels': channels,
            'dtype': str(self.current_frame.dtype),
            'size_bytes': self.current_frame.nbytes,
            'is_connected': self.is_connected
        }
