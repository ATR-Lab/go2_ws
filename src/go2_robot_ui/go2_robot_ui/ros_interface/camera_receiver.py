"""
Camera Receiver Module

Handles camera video stream from Go2 robot with QR code detection capabilities.
Provides real-time video processing with performance monitoring and QR detection overlay.
"""

import cv2
import numpy as np
import time
from typing import Optional, Tuple, List
from python_qt_binding.QtCore import QObject, pyqtSignal, QTimer
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

try:
    from pyzbar import pyzbar
    QR_DETECTION_AVAILABLE = True
except ImportError:
    QR_DETECTION_AVAILABLE = False


class CameraReceiver(QObject):
    """
    Handles camera video stream reception and QR code detection.
    
    This class subscribes to camera image topics and processes frames for
    display and QR code detection, providing performance metrics and
    connection monitoring.
    
    Signals:
        frame_ready: Emitted when a new processed frame is ready (np.ndarray)
        qr_detected: Emitted when QR code is detected (str: qr_data, Point: center)
        performance_update: Emitted with performance metrics (dict)
        connection_lost: Emitted when camera feed is lost
        connection_restored: Emitted when camera feed is restored
    """
    
    # Signals for UI updates
    frame_ready = pyqtSignal(np.ndarray)
    qr_detected = pyqtSignal(str, Point)
    performance_update = pyqtSignal(dict)
    connection_lost = pyqtSignal()
    connection_restored = pyqtSignal()
    
    def __init__(self, node: Node, camera_topic: str):
        """
        Initialize camera receiver.
        
        Args:
            node: ROS2 node for creating subscriptions
            camera_topic: Topic name for camera images
        """
        super().__init__()
        self.node = node
        self.camera_topic = camera_topic
        
        # OpenCV bridge for image conversion
        self.bridge = CvBridge()
        
        # Performance tracking
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.fps = 0.0
        self.last_frame_time = time.time()
        self.latency = 0.0
        
        # Connection monitoring
        self.is_connected = False
        self.last_frame_received = time.time()
        self.connection_timeout = 3.0  # seconds
        
        # QR detection state
        self.qr_detection_enabled = QR_DETECTION_AVAILABLE
        self.last_qr_detection_time = 0
        self.qr_detection_interval = 0.2  # Process QR every 200ms to save CPU
        
        # Set up QoS profile for camera stream
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create camera subscription
        self.camera_sub = node.create_subscription(
            Image,
            camera_topic,
            self._camera_callback,
            self.qos_profile
        )
        
        # Performance update timer
        self.perf_timer = QTimer()
        self.perf_timer.timeout.connect(self._update_performance_metrics)
        self.perf_timer.start(1000)  # Update every second
        
        node.get_logger().info(f'Camera receiver initialized for topic: {camera_topic}')
        if not self.qr_detection_enabled:
            node.get_logger().warn('QR detection disabled - pyzbar not available')
    
    def _camera_callback(self, msg: Image) -> None:
        """Process incoming camera images."""
        try:
            current_time = time.time()
            
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Update connection status
            if not self.is_connected:
                self.is_connected = True
                self.connection_restored.emit()
                self.node.get_logger().info('Camera connection restored')
            
            self.last_frame_received = current_time
            
            # Calculate latency (approximate)
            self.latency = (current_time - self.last_frame_time) * 1000  # ms
            self.last_frame_time = current_time
            
            # Process frame for QR detection
            processed_frame = self._process_frame_for_qr(cv_image)
            
            # Emit processed frame
            self.frame_ready.emit(processed_frame)
            
            # Update frame count for FPS calculation
            self.frame_count += 1
            
        except Exception as e:
            self.node.get_logger().error(f'Error processing camera frame: {e}')
    
    def _process_frame_for_qr(self, frame: np.ndarray) -> np.ndarray:
        """
        Process frame for QR code detection and overlay.
        
        Args:
            frame: Input BGR frame
            
        Returns:
            Processed frame with QR detection overlay
        """
        if not self.qr_detection_enabled:
            return frame
        
        current_time = time.time()
        
        # Only process QR detection at intervals to save CPU
        if current_time - self.last_qr_detection_time > self.qr_detection_interval:
            self.last_qr_detection_time = current_time
            
            # Convert to grayscale for QR detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Detect QR codes
            qr_codes = pyzbar.decode(gray)
            
            for qr_code in qr_codes:
                # Extract QR code data
                qr_data = qr_code.data.decode('utf-8')
                
                # Get bounding box
                points = qr_code.polygon
                if len(points) == 4:
                    # Draw QR code boundary
                    pts = np.array([[p.x, p.y] for p in points], dtype=np.int32)
                    cv2.polylines(frame, [pts], True, (0, 255, 0), 3)
                    
                    # Calculate center point
                    center_x = int(np.mean([p.x for p in points]))
                    center_y = int(np.mean([p.y for p in points]))
                    
                    # Draw center point
                    cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)
                    
                    # Draw QR data text
                    cv2.putText(frame, qr_data, (center_x - 50, center_y - 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    # Emit QR detection signal
                    center_point = Point()
                    center_point.x = float(center_x)
                    center_point.y = float(center_y)
                    center_point.z = 0.0
                    
                    self.qr_detected.emit(qr_data, center_point)
                    
                    self.node.get_logger().info(f'QR Code detected: {qr_data} at ({center_x}, {center_y})')
        
        return frame
    
    def _update_performance_metrics(self) -> None:
        """Update and emit performance metrics."""
        current_time = time.time()
        
        # Calculate FPS
        time_diff = current_time - self.last_fps_time
        if time_diff > 0:
            self.fps = self.frame_count / time_diff
        
        # Reset counters
        self.frame_count = 0
        self.last_fps_time = current_time
        
        # Create performance metrics dictionary
        metrics = {
            'fps': round(self.fps, 1),
            'latency_ms': round(self.latency, 1),
            'connected': self.is_connected,
            'qr_detection': self.qr_detection_enabled
        }
        
        self.performance_update.emit(metrics)
    
    def check_connection_timeout(self) -> None:
        """Check for connection timeout and emit signals accordingly."""
        current_time = time.time()
        
        if self.is_connected and (current_time - self.last_frame_received) > self.connection_timeout:
            self.is_connected = False
            self.connection_lost.emit()
            self.node.get_logger().warn('Camera connection lost - timeout')
    
    def set_qr_detection_enabled(self, enabled: bool) -> None:
        """Enable or disable QR code detection."""
        if QR_DETECTION_AVAILABLE:
            self.qr_detection_enabled = enabled
            self.node.get_logger().info(f'QR detection {"enabled" if enabled else "disabled"}')
        else:
            self.node.get_logger().warn('QR detection not available - pyzbar not installed')
    
    def get_camera_topic(self) -> str:
        """Get the current camera topic."""
        return self.camera_topic
    
    def get_connection_status(self) -> bool:
        """Get current connection status."""
        return self.is_connected