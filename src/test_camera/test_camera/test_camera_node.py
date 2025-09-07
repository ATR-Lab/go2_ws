#!/usr/bin/env python3
"""
Test Camera Node

This node captures video from the PC's camera and publishes it to ROS topics
for testing the human detection and gesture recognition system.

Publishes to:
- /camera/image_raw: Camera feed from PC camera
- /camera/camera_info: Camera calibration info (dummy data)

Parameters:
- camera_index: Index of camera device (default: 0)
- frame_rate: Publishing frame rate (default: 30 Hz)
- image_width: Image width (default: 640)
- image_height: Image height (default: 480)
- flip_horizontal: Flip image horizontally to correct mirror effect (default: True)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import time
from typing import Optional

class TestCameraNode(Node):
    """Test camera node for publishing PC camera feed"""
    
    def __init__(self):
        super().__init__('test_camera_node')
        
        # Initialize parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('enable_preview', True)
        
        # Get parameters
        self.camera_index = self.get_parameter('camera_index').value
        self.frame_rate = self.get_parameter('frame_rate').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.enable_preview = self.get_parameter('enable_preview').value
        
        # Initialize camera
        self.camera = None
        self.bridge = CvBridge()
        
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
        
        # Setup timer for publishing
        self.timer = self.create_timer(
            1.0 / self.frame_rate,  # Convert Hz to seconds
            self.camera_callback
        )
        
        # Initialize camera
        self.initialize_camera()
        
        # Publish camera info once
        self.publish_camera_info()
        
        self.get_logger().info(f'Test Camera Node initialized with camera {self.camera_index}')
        self.get_logger().info(f'Publishing at {self.frame_rate} Hz, resolution: {self.image_width}x{self.image_height}')
    
    def initialize_camera(self):
        """Initialize the camera device"""
        try:
            self.camera = cv2.VideoCapture(self.camera_index)
            
            if not self.camera.isOpened():
                raise RuntimeError(f"Could not open camera {self.camera_index}")
            
            # Set camera properties
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
            self.camera.set(cv2.CAP_PROP_FPS, self.frame_rate)
            
            # Verify settings
            actual_width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.camera.get(cv2.CAP_PROP_FPS)
            
            self.get_logger().info(f'Camera initialized successfully')
            self.get_logger().info(f'Actual resolution: {actual_width}x{actual_height}')
            self.get_logger().info(f'Actual FPS: {actual_fps:.1f}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize camera: {e}')
            self.get_logger().error('Make sure camera is connected and not in use by another application')
            raise
    
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
    
    def camera_callback(self):
        """Timer callback for capturing and publishing camera frames"""
        if self.camera is None or not self.camera.isOpened():
            self.get_logger().warn('Camera not available, skipping frame')
            return
        
        try:
            # Capture frame
            ret, frame = self.camera.read()
            
            if not ret:
                self.get_logger().warn('Failed to capture frame from camera')
                return
            
            # Resize frame if needed
            if frame.shape[1] != self.image_width or frame.shape[0] != self.image_height:
                frame = cv2.resize(frame, (self.image_width, self.image_height))
            
            
            # Convert BGR to RGB (OpenCV uses BGR, ROS expects RGB)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Convert to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(frame_rgb, "rgb8")
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = "camera_frame"
            
            # Publish image
            self.image_pub.publish(ros_image)
            
            # Show preview window if enabled
            if self.enable_preview:
                # Convert back to BGR for OpenCV display
                frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                cv2.imshow('Test Camera Feed', frame_bgr)
                cv2.waitKey(1)  # 1ms delay, non-blocking
            
        except Exception as e:
            self.get_logger().error(f'Error in camera callback: {e}')
    
    def on_shutdown(self):
        """Cleanup when node is shutting down"""
        if self.camera is not None:
            self.camera.release()
        
        if self.enable_preview:
            cv2.destroyAllWindows()
        
        self.get_logger().info('Test Camera Node shutdown complete')


def main(args=None):
    rclpy.init(args=args)
    
    node = TestCameraNode()
    
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