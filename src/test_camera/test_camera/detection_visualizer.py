#!/usr/bin/env python3
"""
Detection Visualizer UI

PyQt5-based visualization interface for the robot dog petting zoo system.
Displays camera feed with YOLO human detection and MediaPipe gesture recognition
overlays in real-time.

Subscribes to:
- /camera/image_raw: Camera feed
- /human_detection/people: Human detection results
- /human_detection/gestures: Gesture recognition results
- /human_detection/pose: Body pose data
- /interaction/state: Interaction state

Features:
- Real-time camera feed display
- YOLO bounding box overlays
- MediaPipe pose landmarks
- Gesture recognition visualization
- Interaction state display
- Performance metrics
"""

import sys
import os
import json
import time
from typing import Dict, List, Optional
import numpy as np
import cv2

# ROS2 Qt binding imports (proper way to integrate Qt with ROS2)
from python_qt_binding.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QPushButton, QTextEdit, 
                             QGroupBox, QGridLayout, QSlider, QCheckBox)
from python_qt_binding.QtCore import QTimer, QThread, pyqtSignal, Qt
from python_qt_binding.QtGui import QPixmap, QImage, QPainter, QPen, QColor, QFont

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

class DetectionVisualizerNode(Node):
    """ROS2 node for detection visualization"""
    
    def __init__(self):
        super().__init__('detection_visualizer_node')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Setup QoS for camera data
        camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
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
        
        self.pose_sub = self.create_subscription(
            String,
            '/human_detection/pose',
            self.pose_callback,
            10
        )
        
        self.state_sub = self.create_subscription(
            String,
            '/interaction/state',
            self.state_callback,
            10
        )
        
        # Initialize state
        self.current_image = None
        self.current_detections = []
        self.current_gestures = []
        self.current_pose = None
        self.current_state = "Unknown"
        self.last_update_time = time.time()
        
        self.get_logger().info('Detection Visualizer Node initialized')


class ImageProcessor(QThread):
    """Thread for processing ROS images and detections"""
    
    image_updated = pyqtSignal(np.ndarray, list, list, dict, str)
    
    def __init__(self, ros_node: DetectionVisualizerNode):
        super().__init__()
        self.ros_node = ros_node
        self.running = True
        
    def run(self):
        """Main processing loop"""
        while self.running:
            if self.ros_node.current_image is not None:
                # Emit processed image with detections
                self.image_updated.emit(
                    self.ros_node.current_image,
                    self.ros_node.current_detections,
                    self.ros_node.current_gestures,
                    self.ros_node.current_pose or {},
                    self.ros_node.current_state
                )
            self.msleep(33)  # ~30 FPS
    
    def stop(self):
        """Stop the processing thread"""
        self.running = False


class DetectionVisualizerUI(QMainWindow):
    """Main PyQt5 visualization window"""
    
    def __init__(self):
        super().__init__()
        self.ros_node = None
        self.image_processor = None
        self.init_ui()
        
    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle('Robot Dog Petting Zoo - Detection Visualizer')
        self.setGeometry(100, 100, 1400, 900)
        
        # Create central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Create main layout
        main_layout = QHBoxLayout(central_widget)
        
        # Left panel - Camera feed and detections
        left_panel = self.create_left_panel()
        main_layout.addWidget(left_panel, stretch=2)
        
        # Right panel - Controls and information
        right_panel = self.create_right_panel()
        main_layout.addWidget(right_panel, stretch=1)
        
        # Setup ROS node and image processor
        self.setup_ros()
        
    def create_left_panel(self):
        """Create the left panel with camera feed and detections"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Camera feed display
        self.camera_label = QLabel()
        self.camera_label.setMinimumSize(640, 480)
        self.camera_label.setStyleSheet("border: 2px solid gray; background-color: black;")
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setText("Waiting for camera feed...")
        layout.addWidget(self.camera_label)
        
        # Detection overlay controls
        overlay_group = QGroupBox("Detection Overlays")
        overlay_layout = QGridLayout(overlay_group)
        
        self.show_bbox_checkbox = QCheckBox("Show Bounding Boxes")
        self.show_bbox_checkbox.setChecked(True)
        overlay_layout.addWidget(self.show_bbox_checkbox, 0, 0)
        
        self.show_pose_checkbox = QCheckBox("Show Pose Landmarks")
        self.show_pose_checkbox.setChecked(True)
        overlay_layout.addWidget(self.show_pose_checkbox, 0, 1)
        
        self.show_gestures_checkbox = QCheckBox("Show Gestures")
        self.show_gestures_checkbox.setChecked(True)
        overlay_layout.addWidget(self.show_gestures_checkbox, 1, 0)
        
        layout.addWidget(overlay_group)
        
        return panel
    
    def create_right_panel(self):
        """Create the right panel with controls and information"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Status information
        status_group = QGroupBox("System Status")
        status_layout = QVBoxLayout(status_group)
        
        self.state_label = QLabel("Interaction State: Unknown")
        self.state_label.setStyleSheet("font-weight: bold; color: blue;")
        status_layout.addWidget(self.state_label)
        
        self.detection_count_label = QLabel("Humans Detected: 0")
        status_layout.addWidget(self.detection_count_label)
        
        self.gesture_count_label = QLabel("Gestures Recognized: 0")
        status_layout.addWidget(self.gesture_count_label)
        
        self.fps_label = QLabel("FPS: 0")
        status_layout.addWidget(self.fps_label)
        
        layout.addWidget(status_group)
        
        # Detection details
        detection_group = QGroupBox("Detection Details")
        detection_layout = QVBoxLayout(detection_group)
        
        self.detection_text = QTextEdit()
        self.detection_text.setMaximumHeight(200)
        self.detection_text.setReadOnly(True)
        detection_layout.addWidget(self.detection_text)
        
        layout.addWidget(detection_group)
        
        # Gesture details
        gesture_group = QGroupBox("Gesture Details")
        gesture_layout = QVBoxLayout(gesture_group)
        
        self.gesture_text = QTextEdit()
        self.gesture_text.setMaximumHeight(150)
        self.gesture_text.setReadOnly(True)
        gesture_layout.addWidget(self.gesture_text)
        
        layout.addWidget(gesture_group)
        
        # Control buttons
        control_group = QGroupBox("Controls")
        control_layout = QVBoxLayout(control_group)
        
        self.clear_button = QPushButton("Clear Display")
        self.clear_button.clicked.connect(self.clear_display)
        control_layout.addWidget(self.clear_button)
        
        self.screenshot_button = QPushButton("Take Screenshot")
        self.screenshot_button.clicked.connect(self.take_screenshot)
        control_layout.addWidget(self.screenshot_button)
        
        layout.addWidget(control_group)
        
        # Add stretch to push everything to the top
        layout.addStretch()
        
        return panel
    
    def setup_ros(self):
        """Setup ROS node and image processor"""
        try:
            # Initialize ROS node
            self.ros_node = DetectionVisualizerNode()
            
            # Create image processor thread
            self.image_processor = ImageProcessor(self.ros_node)
            self.image_processor.image_updated.connect(self.update_display)
            self.image_processor.start()
            
            # Setup timer for ROS spinning
            self.ros_timer = QTimer()
            self.ros_timer.timeout.connect(self.spin_ros)
            self.ros_timer.start(10)  # 100 Hz
            
            # Setup FPS timer
            self.fps_timer = QTimer()
            self.fps_timer.timeout.connect(self.update_fps)
            self.fps_timer.start(1000)  # 1 Hz
            
            self.frame_count = 0
            self.last_fps_time = time.time()
            
        except Exception as e:
            print(f"Failed to setup ROS: {e}")
    
    def spin_ros(self):
        """Spin ROS node to process callbacks"""
        if self.ros_node:
            rclpy.spin_once(self.ros_node, timeout_sec=0.001)
    
    def update_fps(self):
        """Update FPS display"""
        current_time = time.time()
        if current_time - self.last_fps_time > 0:
            fps = self.frame_count / (current_time - self.last_fps_time)
            self.fps_label.setText(f"FPS: {fps:.1f}")
            self.frame_count = 0
            self.last_fps_time = current_time
    
    def image_callback(self, msg: Image):
        """Handle incoming camera images"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.ros_node.bridge.imgmsg_to_cv2(msg, "rgb8")
            self.ros_node.current_image = cv_image
            self.frame_count += 1
        except Exception as e:
            print(f"Error processing image: {e}")
    
    def people_callback(self, msg: String):
        """Handle human detection results"""
        try:
            data = json.loads(msg.data)
            self.ros_node.current_detections = [data]  # Wrap in list for consistency
            self.update_detection_display(data)
        except Exception as e:
            print(f"Error processing people data: {e}")
    
    def gestures_callback(self, msg: String):
        """Handle gesture recognition results"""
        try:
            data = json.loads(msg.data)
            self.ros_node.current_gestures = data.get('gestures', [])
            self.update_gesture_display(data)
        except Exception as e:
            print(f"Error processing gestures: {e}")
    
    def pose_callback(self, msg: String):
        """Handle pose data"""
        try:
            data = json.loads(msg.data)
            self.ros_node.current_pose = data
        except Exception as e:
            print(f"Error processing pose data: {e}")
    
    def state_callback(self, msg: String):
        """Handle interaction state updates"""
        try:
            data = json.loads(msg.data)
            self.ros_node.current_state = data.get('current_state', 'Unknown')
            self.state_label.setText(f"Interaction State: {self.ros_node.current_state}")
        except Exception as e:
            print(f"Error processing state data: {e}")
    
    def update_detection_display(self, detection_data: Dict):
        """Update detection information display"""
        try:
            detection = detection_data.get('detection', {})
            confidence = detection.get('confidence', 0.0)
            distance = detection_data.get('distance', 0.0)
            zone = detection_data.get('zone', 'unknown')
            
            text = f"Human Detected:\n"
            text += f"Confidence: {confidence:.2f}\n"
            text += f"Distance: {distance:.2f}m\n"
            text += f"Zone: {zone}\n"
            
            self.detection_text.setText(text)
            self.detection_count_label.setText(f"Humans Detected: 1")
            
        except Exception as e:
            print(f"Error updating detection display: {e}")
    
    def update_gesture_display(self, gesture_data: Dict):
        """Update gesture information display"""
        try:
            gestures = gesture_data.get('gestures', [])
            text = f"Gestures Recognized:\n"
            for gesture in gestures:
                text += f"• {gesture}\n"
            
            self.gesture_text.setText(text)
            self.gesture_count_label.setText(f"Gestures Recognized: {len(gestures)}")
            
        except Exception as e:
            print(f"Error updating gesture display: {e}")
    
    def update_display(self, image: np.ndarray, detections: List, gestures: List, 
                      pose: Dict, state: str):
        """Update the main display with processed image and detections"""
        try:
            # Create a copy of the image for drawing
            display_image = image.copy()
            
            # Draw detections if enabled
            if self.show_bbox_checkbox.isChecked() and detections:
                for detection in detections:
                    self.draw_detection_bbox(display_image, detection)
            
            # Draw pose landmarks if enabled
            if self.show_pose_checkbox.isChecked() and pose:
                self.draw_pose_landmarks(display_image, pose)
            
            # Draw gesture information if enabled
            if self.show_gestures_checkbox.isChecked() and gestures:
                self.draw_gesture_info(display_image, gestures)
            
            # Convert to QPixmap and display
            self.display_image(display_image)
            
        except Exception as e:
            print(f"Error updating display: {e}")
    
    def draw_detection_bbox(self, image: np.ndarray, detection_data: Dict):
        """Draw bounding box for human detection"""
        try:
            detection = detection_data.get('detection', {})
            bbox = detection.get('bbox', [])
            confidence = detection.get('confidence', 0.0)
            
            if len(bbox) == 4:
                x1, y1, x2, y2 = bbox
                
                # Draw bounding box
                color = (0, 255, 0)  # Green
                thickness = 2
                cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness)
                
                # Draw confidence label
                label = f"Human: {confidence:.2f}"
                cv2.putText(image, label, (x1, y1-10), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.5, color, 1)
                
        except Exception as e:
            print(f"Error drawing detection bbox: {e}")
    
    def draw_pose_landmarks(self, image: np.ndarray, pose_data: Dict):
        """Draw MediaPipe pose landmarks"""
        try:
            # This would draw pose landmarks if we had the actual pose data
            # For now, just add a placeholder
            cv2.putText(image, "Pose Data Available", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
        except Exception as e:
            print(f"Error drawing pose landmarks: {e}")
    
    def draw_gesture_info(self, image: np.ndarray, gestures: List):
        """Draw gesture information on image"""
        try:
            if gestures:
                y_offset = 60
                for i, gesture in enumerate(gestures):
                    text = f"Gesture: {gesture}"
                    cv2.putText(image, text, (10, y_offset + i*25), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
        except Exception as e:
            print(f"Error drawing gesture info: {e}")
    
    def display_image(self, image: np.ndarray):
        """Display OpenCV image in PyQt5 label"""
        try:
            # Convert RGB to BGR for OpenCV
            image_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            
            # Convert to QImage
            height, width, channel = image_bgr.shape
            bytes_per_line = 3 * width
            q_image = QImage(image_bgr.data, width, height, bytes_per_line, QImage.Format_RGB888)
            
            # Convert to QPixmap and scale to fit label
            pixmap = QPixmap.fromImage(q_image)
            scaled_pixmap = pixmap.scaled(self.camera_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            
            # Display in label
            self.camera_label.setPixmap(scaled_pixmap)
            
        except Exception as e:
            print(f"Error displaying image: {e}")
    
    def clear_display(self):
        """Clear the detection and gesture displays"""
        self.detection_text.clear()
        self.gesture_text.clear()
        self.detection_count_label.setText("Humans Detected: 0")
        self.gesture_count_label.setText("Gestures Recognized: 0")
    
    def take_screenshot(self):
        """Take a screenshot of the current display"""
        try:
            if self.ros_node and self.ros_node.current_image is not None:
                timestamp = int(time.time())
                filename = f"detection_screenshot_{timestamp}.png"
                cv2.imwrite(filename, self.ros_node.current_image)
                print(f"Screenshot saved as {filename}")
        except Exception as e:
            print(f"Error taking screenshot: {e}")
    
    def closeEvent(self, event):
        """Handle window close event"""
        if self.image_processor:
            self.image_processor.stop()
            self.image_processor.wait()
        
        if self.ros_node:
            self.ros_node.destroy_node()
        
        event.accept()


def main(args=None):
    # Initialize ROS
    rclpy.init(args=args)
    
    # Create Qt application
    app = QApplication(sys.argv)
    
    # Create and show the main window
    window = DetectionVisualizerUI()
    window.show()
    
    # Start Qt event loop
    sys.exit(app.exec_())


if __name__ == '__main__':
    main() 