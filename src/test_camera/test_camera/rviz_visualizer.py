#!/usr/bin/env python3
"""
RViz2 Visualizer Node

Simple ROS2 node that publishes visualization markers for human detection results.
This avoids PyQt5 conflicts by using ROS2's built-in RViz2 visualization.

Publishes to:
- /visualization_marker_array: Detection bounding boxes and labels
- /tf: Transform frames for visualization

Usage:
1. Start this node: ros2 run test_camera rviz_visualizer
2. Start RViz2: rviz2
3. Add MarkerArray display and set topic to /visualization_marker_array
4. Add TF display to see coordinate frames
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Vector3, Quaternion
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import json
import time
from typing import Dict, List

class RVizVisualizerNode(Node):
    """RViz2-based visualization node for human detection results"""
    
    def __init__(self):
        super().__init__('rviz_visualizer_node')
        
        # Initialize TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Setup QoS for visualization markers
        marker_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Setup publishers
        self.marker_pub = self.create_publisher(
            MarkerArray, 
            '/visualization_marker_array', 
            marker_qos
        )
        
        # Setup subscribers
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
        
        # Initialize state
        self.current_detections = []
        self.current_gestures = []
        self.current_state = "Unknown"
        self.marker_id_counter = 0
        
        # Setup timer for TF publishing
        self.tf_timer = self.create_timer(
            0.1,  # 10 Hz
            self.publish_tf
        )
        
        self.get_logger().info('RViz2 Visualizer Node initialized')
        self.get_logger().info('Start RViz2 and add MarkerArray display for /visualization_marker_array')
    
    def people_callback(self, msg: String):
        """Handle human detection results"""
        try:
            data = json.loads(msg.data)
            self.current_detections = [data]  # Wrap in list for consistency
            self.publish_detection_markers()
        except Exception as e:
            self.get_logger().error(f'Error processing people data: {e}')
    
    def gestures_callback(self, msg: String):
        """Handle gesture recognition results"""
        try:
            data = json.loads(msg.data)
            self.current_gestures = data.get('gestures', [])
            self.publish_gesture_markers()
        except Exception as e:
            self.get_logger().error(f'Error processing gestures: {e}')
    
    def state_callback(self, msg: String):
        """Handle interaction state updates"""
        try:
            data = json.loads(msg.data)
            self.current_state = data.get('current_state', 'Unknown')
            self.get_logger().info(f'Interaction State: {self.current_state}')
        except Exception as e:
            self.get_logger().error(f'Error processing state data: {e}')
    
    def publish_detection_markers(self):
        """Publish detection visualization markers"""
        try:
            marker_array = MarkerArray()
            
            for detection_data in self.current_detections:
                detection = detection_data.get('detection', {})
                bbox = detection.get('bbox', [])
                confidence = detection.get('confidence', 0.0)
                distance = detection_data.get('distance', 0.0)
                zone = detection_data.get('zone', 'unknown')
                
                if len(bbox) == 4:
                    x1, y1, x2, y2 = bbox
                    
                    # Create bounding box marker
                    bbox_marker = self.create_bbox_marker(x1, y1, x2, y2, confidence)
                    marker_array.markers.append(bbox_marker)
                    
                    # Create label marker
                    label_marker = self.create_label_marker(x1, y1, confidence, distance, zone)
                    marker_array.markers.append(label_marker)
            
            # Publish markers
            self.marker_pub.publish(marker_array)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing detection markers: {e}')
    
    def publish_gesture_markers(self):
        """Publish gesture visualization markers"""
        try:
            if not self.current_gestures:
                return
            
            marker_array = MarkerArray()
            
            for i, gesture in enumerate(self.current_gestures):
                # Create gesture text marker
                gesture_marker = self.create_gesture_marker(gesture, i)
                marker_array.markers.append(gesture_marker)
            
            # Publish markers
            self.marker_pub.publish(marker_array)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing gesture markers: {e}')
    
    def create_bbox_marker(self, x1: int, y1: int, x2: int, y2: int, confidence: float) -> Marker:
        """Create a bounding box marker"""
        marker = Marker()
        marker.header.frame_id = "camera_frame"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "human_detections"
        marker.id = self.marker_id_counter
        self.marker_id_counter += 1
        
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        # Set bounding box points
        marker.points = [
            Point(x=x1, y=y1, z=0.0),
            Point(x=x2, y=y1, z=0.0),
            Point(x=x2, y=y2, z=0.0),
            Point(x=x1, y=y2, z=0.0),
            Point(x=x1, y=y1, z=0.0)  # Close the box
        ]
        
        # Set appearance
        marker.scale.x = 2.0  # Line width
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        return marker
    
    def create_label_marker(self, x: int, y: int, confidence: float, distance: float, zone: str) -> Marker:
        """Create a text label marker"""
        marker = Marker()
        marker.header.frame_id = "camera_frame"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "detection_labels"
        marker.id = self.marker_id_counter
        self.marker_id_counter += 1
        
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Set position
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y - 20)  # Above the bounding box
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Set text
        marker.text = f"Human: {confidence:.2f}\nDist: {distance:.1f}m\nZone: {zone}"
        
        # Set appearance
        marker.scale.z = 20.0  # Text size
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        return marker
    
    def create_gesture_marker(self, gesture: str, index: int) -> Marker:
        """Create a gesture text marker"""
        marker = Marker()
        marker.header.frame_id = "camera_frame"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "gestures"
        marker.id = self.marker_id_counter
        self.marker_id_counter += 1
        
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Set position (top right of screen)
        marker.pose.position.x = 500.0
        marker.pose.position.y = 400.0 - (index * 30)
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Set text
        marker.text = f"Gesture: {gesture}"
        
        # Set appearance
        marker.scale.z = 25.0  # Text size
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        return marker
    
    def publish_tf(self):
        """Publish camera frame transform"""
        try:
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = "map"
            transform.child_frame_id = "camera_frame"
            
            # Set camera position (adjust as needed)
            transform.transform.translation.x = 0.0
            transform.transform.translation.y = 0.0
            transform.transform.translation.z = 0.0
            
            # Set camera orientation (looking forward)
            transform.transform.rotation.w = 1.0
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            
            # Publish transform
            self.tf_broadcaster.sendTransform(transform)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing TF: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = RVizVisualizerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 