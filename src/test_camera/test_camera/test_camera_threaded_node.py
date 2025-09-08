#!/usr/bin/env python3
"""
Threaded Test Camera Node

This node implements a multi-threaded camera capture approach inspired by coffee_vision
to eliminate camera hardware blocking issues. It uses dedicated threads for:
- Camera capture (isolated from ROS2 executor)
- Frame processing and publishing (controlled rate)

This solves the FPS drop issues caused by camera hardware blocking the ROS2 executor
in timer callbacks, particularly on Intel systems.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import time
import threading
from typing import Optional


class TestCameraThreadedNode(Node):
    """Multi-threaded test camera node for improved performance"""
    
    def __init__(self):
        super().__init__('test_camera_threaded_node')
        
        # Initialize parameters (same as original)
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('enable_preview', False)  # Default disabled for performance
        
        # Get parameters
        self.camera_index = self.get_parameter('camera_index').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.enable_preview = self.get_parameter('enable_preview').value
        
        # Initialize camera and bridge
        self.camera = None
        self.bridge = CvBridge()
        
        # Threading components
        self.running = False
        self.current_frame = None
        self.frame_timestamp = 0  # Track frame freshness
        self.frame_lock = threading.Lock()
        self.capture_thread = None
        self.publish_thread = None
        
        # Performance monitoring
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.capture_fps = 0.0
        
        # Setup QoS for camera data - use best effort to match Go2 robot default
        camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Setup publishers
        self.image_pub = self.create_publisher(
            Image, 
            '/camera/image_raw', 
            camera_qos
        )
        
        self.camera_info_pub = self.create_publisher(
            CameraInfo, 
            '/camera/camera_info', 
            10
        )
        
        # Initialize camera and start threads
        self.initialize_camera()
        self.start_capture_threads()
        
        # Publish camera info once
        self.publish_camera_info()
        
        self.get_logger().info(f'Threaded Test Camera Node initialized with camera {self.camera_index}')
        self.get_logger().info(f'Target publishing rate: {self.frame_rate} Hz, resolution: {self.image_width}x{self.image_height}')
        self.get_logger().info('Using multi-threaded architecture for improved performance')
    
    def initialize_camera(self):
        """Initialize the camera device with optimization"""
        try:
            # Try different backends if the default doesn't work
            backends_to_try = [
                (cv2.CAP_V4L2, "V4L2"),
                (cv2.CAP_GSTREAMER, "GStreamer"),
                (cv2.CAP_ANY, "Auto-detect")
            ]
            
            success = False
            error_msg = ""
            
            # Try each backend until one works
            for backend, backend_name in backends_to_try:
                try:
                    if backend == cv2.CAP_ANY:
                        self.camera = cv2.VideoCapture(self.camera_index)
                    else:
                        self.camera = cv2.VideoCapture(self.camera_index, backend)
                    
                    if not self.camera.isOpened():
                        error_msg = f"Could not open camera {self.camera_index} with {backend_name} backend"
                        continue
                    
                    success = True
                    self.get_logger().info(f"Successfully opened camera with {backend_name} backend")
                    break
                except Exception as e:
                    error_msg = f"Error opening camera with {backend_name} backend: {str(e)}"
                    continue
            
            if not success:
                raise RuntimeError(f"Failed to open camera: {error_msg}")
            
            # Set camera properties with optimization
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
            self.camera.set(cv2.CAP_PROP_FPS, self.frame_rate)
            
            # Performance optimizations from coffee_vision
            self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimal latency
            
            # Try to set MJPEG codec for hardware acceleration
            fourcc = cv2.VideoWriter_fourcc('M','J','P','G')
            self.camera.set(cv2.CAP_PROP_FOURCC, fourcc)
            
            # Verify settings
            actual_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.camera.get(cv2.CAP_PROP_FPS)
            
            self.get_logger().info(f'Camera initialized successfully')
            self.get_logger().info(f'Actual resolution: {actual_width}x{actual_height}')
            self.get_logger().info(f'Actual FPS: {actual_fps:.1f}')
            
            # Warm up the camera
            self.get_logger().info('Warming up camera...')
            for _ in range(5):
                self.camera.read()
            
            # Read a test frame to check actual size
            ret, frame = self.camera.read()
            if ret:
                frame_h, frame_w = frame.shape[:2]
                if frame_w != actual_width or frame_h != actual_height:
                    self.get_logger().warn(f"Actual frame size ({frame_w}x{frame_h}) differs from requested ({actual_width}x{actual_height})")
                    self.image_width = frame_w
                    self.image_height = frame_h
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize camera: {e}')
            self.get_logger().error('Make sure camera is connected and not in use by another application')
            raise
    
    def start_capture_threads(self):
        """Start the camera capture and publishing threads"""
        self.running = True
        
        # Start capture thread (dedicated camera hardware access)
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()
        
        # Start publishing thread (controlled ROS2 publishing)
        self.publish_thread = threading.Thread(target=self._publish_loop, daemon=True)
        self.publish_thread.start()
        
        self.get_logger().info('Camera capture and publishing threads started')
    
    def _capture_loop(self):
        """Dedicated camera capture thread - runs independently of ROS2 executor"""
        self.get_logger().info('Camera capture thread started')
        
        while self.running:
            if self.camera is None or not self.camera.isOpened():
                time.sleep(0.1)
                continue
            
            try:
                # Capture frame - this may block, but only affects this thread
                capture_start = time.time()
                ret, frame = self.camera.read()
                capture_time = time.time() - capture_start
                
                if not ret:
                    self.get_logger().warn('Failed to capture frame from camera')
                    time.sleep(0.01)  # Brief pause before retry
                    continue
                
                # Log slow captures for debugging
                if capture_time > 0.1:  # > 100ms is problematic
                    self.get_logger().warn(f'Slow camera capture: {capture_time*1000:.1f}ms')
                
                # Resize frame if needed
                if frame.shape[1] != self.image_width or frame.shape[0] != self.image_height:
                    frame = cv2.resize(frame, (self.image_width, self.image_height))
                
                # Update shared frame buffer (thread-safe)
                with self.frame_lock:
                    self.current_frame = frame  # Direct assignment, no copy
                    self.frame_timestamp = time.time()
                
                # Update performance metrics
                self.frame_count += 1
                current_time = time.time()
                if current_time - self.last_fps_time >= 1.0:
                    self.capture_fps = self.frame_count / (current_time - self.last_fps_time)
                    self.frame_count = 0
                    self.last_fps_time = current_time
                    self.get_logger().debug(f'Capture FPS: {self.capture_fps:.1f}')
                
                # Show preview if enabled (in capture thread to avoid blocking)
                if self.enable_preview and frame is not None:
                    cv2.imshow('Threaded Camera Feed', frame)
                    cv2.waitKey(1)  # Non-blocking in dedicated thread
                
            except Exception as e:
                self.get_logger().error(f'Error in capture loop: {e}')
                time.sleep(0.1)  # Brief pause before retry
    
    def _publish_loop(self):
        """Dedicated publishing thread - controlled rate ROS2 publishing"""
        self.get_logger().info('Publishing thread started')
        
        min_publish_interval = 1.0 / self.frame_rate
        last_publish_time = 0
        
        while self.running:
            current_time = time.time()
            
            # Check if enough time has passed since last publish
            if current_time - last_publish_time < min_publish_interval:
                time.sleep(0.001)  # Brief sleep to avoid busy waiting
                continue
            
            # Get latest frame (thread-safe)
            frame = None
            frame_time = 0
            with self.frame_lock:
                if self.current_frame is not None:
                    frame = self.current_frame  # Direct assignment, no copy
                    frame_time = self.frame_timestamp
            
            if frame is None:
                time.sleep(0.01)
                continue
            
            # Skip stale frames (older than 100ms)
            if current_time - frame_time > 0.1:
                continue
            
            try:
                # Publish BGR directly to match Go2 robot encoding (no conversion needed)
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                ros_image.header.stamp = self.get_clock().now().to_msg()
                ros_image.header.frame_id = "camera_frame"
                
                # Publish image
                self.image_pub.publish(ros_image)
                last_publish_time = current_time
                
            except Exception as e:
                self.get_logger().error(f'Error publishing frame: {e}')
    
    def publish_camera_info(self):
        """Publish camera calibration info (dummy data for testing)"""
        try:
            camera_info = CameraInfo()
            camera_info.header.frame_id = "camera_frame"
            camera_info.header.stamp = self.get_clock().now().to_msg()
            
            # Set image dimensions
            camera_info.width = self.image_width
            camera_info.height = self.image_height
            
            # Dummy calibration matrix (identity matrix for testing)
            camera_info.k = [
                float(self.image_width), 0.0, float(self.image_width / 2.0),
                0.0, float(self.image_height), float(self.image_height / 2.0),
                0.0, 0.0, 1.0
            ]
            
            # Dummy distortion coefficients (no distortion)
            camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            
            # Dummy projection matrix
            camera_info.p = [
                float(self.image_width), 0.0, float(self.image_width / 2.0), 0.0,
                0.0, float(self.image_height), float(self.image_height / 2.0), 0.0,
                0.0, 0.0, 1.0, 0.0
            ]
            
            self.camera_info_pub.publish(camera_info)
            self.get_logger().debug('Published camera info')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing camera info: {e}')
    
    def on_shutdown(self):
        """Cleanup when node is shutting down"""
        self.get_logger().info('Shutting down threaded camera node...')
        
        # Stop threads
        self.running = False
        
        # Wait for threads to finish (with timeout)
        if self.capture_thread and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2.0)
        
        if self.publish_thread and self.publish_thread.is_alive():
            self.publish_thread.join(timeout=2.0)
        
        # Release camera
        if self.camera is not None:
            self.camera.release()
        
        # Close preview window if enabled
        if self.enable_preview:
            cv2.destroyAllWindows()
        
        self.get_logger().info('Threaded camera node shutdown complete')


def main(args=None):
    rclpy.init(args=args)
    
    node = TestCameraThreadedNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
