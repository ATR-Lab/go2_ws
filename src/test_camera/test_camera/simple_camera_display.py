#!/usr/bin/env python3
"""
Simple Camera Display Node

A straightforward camera display that shows the USB camera feed with
YOLO human detection and MediaPipe gesture recognition overlays.
Uses OpenCV's built-in display to avoid Qt conflicts.

Usage:
    ros2 run test_camera simple_camera_display
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import time
from typing import Dict, List, Optional

class SimpleCameraDisplay(Node):
    """Simple camera display with YOLO and MediaPipe overlays"""
    
    def __init__(self):
        super().__init__('simple_camera_display')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Camera state
        self.current_image = None
        self.current_detections = []
        self.current_gestures = []
        self.current_state = "Unknown"
        
        # Performance tracking
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.fps = 0.0
        
        # Setup QoS for camera data
        camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Setup subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            camera_qos
        )
        
        self.people_sub = self.create_subscription(
            String,
            '/human_detection/people',
            self.people_callback,
            10
        )
        
        self.gestures_sub = self.create_subscription(
            String,
            '/human_detection/gestures',
            self.gestures_callback,
            10
        )
        
        self.state_sub = self.create_subscription(
            String,
            '/interaction/state',
            self.state_callback,
            10
        )
        
        # Setup display timer
        self.display_timer = self.create_timer(
            0.033,  # ~30 FPS
            self.update_display
        )
        
        self.get_logger().info('Simple Camera Display Node initialized')
        self.get_logger().info('Press "q" to quit, "s" to save screenshot')
    
    def image_callback(self, msg: Image):
        """Handle incoming camera images"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_image = cv_image
            
            # Update FPS
            self.frame_count += 1
            current_time = time.time()
            if current_time - self.last_fps_time >= 1.0:
                self.fps = self.frame_count / (current_time - self.last_fps_time)
                self.frame_count = 0
                self.last_fps_time = current_time
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def people_callback(self, msg: String):
        """Handle human detection results"""
        try:
            data = json.loads(msg.data)
            self.current_detections = [data]  # Wrap in list for consistency
        except Exception as e:
            self.get_logger().error(f'Error processing people data: {e}')
    
    def gestures_callback(self, msg: String):
        """Handle gesture recognition results"""
        try:
            data = json.loads(msg.data)
            self.current_gestures = data.get('gestures', [])
        except Exception as e:
            self.get_logger().error(f'Error processing gestures: {e}')
    
    def state_callback(self, msg: String):
        """Handle interaction state updates"""
        try:
            data = json.loads(msg.data)
            self.current_state = data.get('current_state', 'Unknown')
        except Exception as e:
            self.get_logger().error(f'Error processing state data: {e}')
    
    def update_display(self):
        """Update the camera display with overlays"""
        if self.current_image is None:
            return
        
        try:
            # Create a copy for display
            display_image = self.current_image.copy()
            
            # Draw YOLO detections
            self.draw_yolo_detections(display_image)
            
            # Draw MediaPipe gestures
            self.draw_gesture_info(display_image)
            
            # Draw interaction state
            self.draw_state_info(display_image)
            
            # Draw performance info
            self.draw_performance_info(display_image)
            
            # Display the image
            cv2.imshow('Robot Dog Petting Zoo - Camera Feed', display_image)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Quit requested by user')
                rclpy.shutdown()
            elif key == ord('s'):
                self.save_screenshot(display_image)
                
        except Exception as e:
            self.get_logger().error(f'Error updating display: {e}')
    
    def draw_yolo_detections(self, image: np.ndarray):
        """Draw YOLO human detection bounding boxes"""
        try:
            for detection_data in self.current_detections:
                detection = detection_data.get('detection', {})
                bbox = detection.get('bbox', [])
                confidence = detection.get('confidence', 0.0)
                distance = detection_data.get('distance', 0.0)
                zone = detection_data.get('zone', 'unknown')
                
                if len(bbox) == 4:
                    x1, y1, x2, y2 = bbox
                    
                    # Draw bounding box
                    color = (0, 255, 0)  # Green for humans
                    cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
                    
                    # Draw label
                    label = f"Human: {confidence:.2f}"
                    label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                    cv2.rectangle(image, (x1, y1 - label_size[1] - 10), 
                                (x1 + label_size[0], y1), color, -1)
                    cv2.putText(image, label, (x1, y1 - 5), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    
                    # Draw distance and zone info
                    info_text = f"Dist: {distance:.1f}m | Zone: {zone}"
                    cv2.putText(image, info_text, (x1, y2 + 20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                    
        except Exception as e:
            self.get_logger().error(f'Error drawing YOLO detections: {e}')
    
    def draw_gesture_info(self, image: np.ndarray):
        """Draw MediaPipe gesture recognition results"""
        try:
            if self.current_gestures:
                y_offset = 30
                for i, gesture in enumerate(self.current_gestures):
                    text = f"Gesture: {gesture}"
                    cv2.putText(image, text, (10, y_offset + i*25), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    
        except Exception as e:
            self.get_logger().error(f'Error drawing gesture info: {e}')
    
    def draw_state_info(self, image: np.ndarray):
        """Draw interaction state information"""
        try:
            # Draw state in top-right corner
            state_text = f"State: {self.current_state}"
            text_size = cv2.getTextSize(state_text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)[0]
            x = image.shape[1] - text_size[0] - 20
            y = 30
            
            # Background rectangle
            cv2.rectangle(image, (x - 10, y - text_size[1] - 10), 
                         (x + text_size[0] + 10, y + 10), (0, 0, 0), -1)
            
            # State text
            color = (0, 255, 0) if self.current_state == "patrol" else (255, 255, 0)
            cv2.putText(image, state_text, (x, y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            
        except Exception as e:
            self.get_logger().error(f'Error drawing state info: {e}')
    
    def draw_performance_info(self, image: np.ndarray):
        """Draw performance metrics"""
        try:
            # FPS counter
            fps_text = f"FPS: {self.fps:.1f}"
            cv2.putText(image, fps_text, (10, image.shape[0] - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Detection count
            detection_count = len(self.current_detections)
            detection_text = f"Humans: {detection_count}"
            cv2.putText(image, detection_text, (10, image.shape[0] - 40), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            # Gesture count
            gesture_count = len(self.current_gestures)
            gesture_text = f"Gestures: {gesture_count}"
            cv2.putText(image, gesture_text, (10, image.shape[0] - 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
        except Exception as e:
            self.get_logger().error(f'Error drawing performance info: {e}')
    
    def save_screenshot(self, image: np.ndarray):
        """Save a screenshot of the current display"""
        try:
            timestamp = int(time.time())
            filename = f"camera_screenshot_{timestamp}.png"
            cv2.imwrite(filename, image)
            self.get_logger().info(f'Screenshot saved as {filename}')
        except Exception as e:
            self.get_logger().error(f'Error saving screenshot: {e}')
    
    def on_shutdown(self):
        """Clean up on shutdown"""
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleCameraDisplay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        try:
            node.on_shutdown()
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main() 