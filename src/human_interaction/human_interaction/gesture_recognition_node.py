#!/usr/bin/env python3
"""
Gesture Recognition Node

This node provides advanced gesture recognition capabilities for the robot dog petting zoo.
It analyzes hand landmarks and body poses to recognize various human gestures and
provides detailed gesture analysis for interaction responses.

Subscribes to:
- /human_detection/pose: Body pose data from MediaPipe
- /human_detection/gestures: Basic gesture data

Publishes to:
- /gesture_recognition/detailed_gestures: Detailed gesture analysis
- /gesture_recognition/gesture_sequences: Gesture sequence patterns
- /gesture_recognition/gesture_confidence: Gesture confidence scores
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
import json
import time
import numpy as np
from typing import Dict, List, Optional, Tuple
import math

class GestureRecognitionNode(Node):
    """Advanced gesture recognition and analysis node"""
    
    def __init__(self):
        super().__init__('gesture_recognition_node')
        
        # Initialize parameters
        self.declare_parameters()
        
        # Initialize gesture recognition state
        self.gesture_history = []
        self.current_gestures = []
        self.gesture_sequences = []
        self.last_gesture_time = time.time()
        
        # Gesture recognition parameters
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.gesture_timeout = self.get_parameter('gesture_timeout').value
        self.sequence_length = self.get_parameter('sequence_length').value
        
        # Setup publishers
        self.setup_publishers()
        
        # Setup subscribers
        self.setup_subscribers()
        
        # Setup timers
        self.setup_timers()
        
        self.get_logger().info('Gesture Recognition Node initialized')
    
    def declare_parameters(self):
        """Declare node parameters"""
        self.declare_parameter('confidence_threshold', 0.7)
        self.declare_parameter('gesture_timeout', 2.0)  # seconds
        self.declare_parameter('sequence_length', 5)  # gestures
        self.declare_parameter('enable_advanced_gestures', True)
        self.declare_parameter('enable_gesture_sequences', True)
        self.declare_parameter('enable_confidence_scoring', True)
    
    def setup_publishers(self):
        """Setup ROS publishers"""
        # Detailed gesture analysis
        self.detailed_gestures_pub = self.create_publisher(
            String, 
            '/gesture_recognition/detailed_gestures', 
            10
        )
        
        # Gesture sequences
        self.gesture_sequences_pub = self.create_publisher(
            String, 
            '/gesture_recognition/gesture_sequences', 
            10
        )
        
        # Gesture confidence scores
        self.gesture_confidence_pub = self.create_publisher(
            String, 
            '/gesture_recognition/gesture_confidence', 
            10
        )
    
    def setup_subscribers(self):
        """Setup ROS subscribers"""
        # Body pose data
        self.pose_sub = self.create_subscription(
            PoseArray,
            '/human_detection/pose',
            self.pose_callback,
            10
        )
        
        # Basic gesture data
        self.gestures_sub = self.create_subscription(
            String,
            '/human_detection/gestures',
            self.gestures_callback,
            10
        )
    
    def setup_timers(self):
        """Setup ROS timers"""
        # Gesture analysis timer
        self.analysis_timer = self.create_timer(
            0.5,  # 2 Hz
            self.gesture_analysis_callback
        )
        
        # Gesture cleanup timer
        self.cleanup_timer = self.create_timer(
            1.0,  # 1 Hz
            self.gesture_cleanup_callback
        )
    
    def pose_callback(self, msg: PoseArray):
        """Handle body pose data for gesture analysis"""
        try:
            # Extract pose landmarks
            landmarks = self.extract_landmarks_from_pose_array(msg)
            
            if landmarks:
                # Analyze pose for gestures
                pose_gestures = self.analyze_pose_gestures(landmarks)
                
                # Update current gestures
                self.update_gestures(pose_gestures, 'pose')
            
        except Exception as e:
            self.get_logger().error(f'Error processing pose data: {e}')
    
    def gestures_callback(self, msg: String):
        """Handle basic gesture data"""
        try:
            data = json.loads(msg.data)
            gestures = data.get('gestures', [])
            
            if gestures:
                # Analyze basic gestures
                basic_gestures = self.analyze_basic_gestures(gestures)
                
                # Update current gestures
                self.update_gestures(basic_gestures, 'basic')
            
        except Exception as e:
            self.get_logger().error(f'Error processing gesture data: {e}')
    
    def extract_landmarks_from_pose_array(self, pose_array: PoseArray) -> Optional[Dict]:
        """Extract landmark data from PoseArray message"""
        try:
            if len(pose_array.poses) < 7:  # Need at least key landmarks
                return None
            
            landmarks = {}
            
            # Map pose array indices to landmark names
            landmark_mapping = {
                0: 'nose',
                1: 'left_shoulder',
                2: 'right_shoulder',
                3: 'left_elbow',
                4: 'right_elbow',
                5: 'left_wrist',
                6: 'right_wrist'
            }
            
            for i, pose in enumerate(pose_array.poses[:7]):
                if i in landmark_mapping:
                    landmarks[landmark_mapping[i]] = {
                        'x': pose.position.x,
                        'y': pose.position.y,
                        'z': pose.position.z
                    }
            
            return landmarks
            
        except Exception as e:
            self.get_logger().error(f'Error extracting landmarks: {e}')
            return None
    
    def analyze_pose_gestures(self, landmarks: Dict) -> List[Dict]:
        """Analyze pose landmarks for gesture recognition"""
        gestures = []
        
        try:
            # Check for waving gesture
            if self.detect_waving_gesture(landmarks):
                gestures.append({
                    'type': 'wave',
                    'confidence': 0.8,
                    'source': 'pose',
                    'timestamp': time.time()
                })
            
            # Check for pointing gesture
            if self.detect_pointing_gesture(landmarks):
                gestures.append({
                    'type': 'point',
                    'confidence': 0.7,
                    'source': 'pose',
                    'timestamp': time.time()
                })
            
            # Check for raised hands gesture
            if self.detect_raised_hands_gesture(landmarks):
                gestures.append({
                    'type': 'raised_hands',
                    'confidence': 0.6,
                    'source': 'pose',
                    'timestamp': time.time()
                })
            
            # Check for clapping gesture
            if self.detect_clapping_gesture(landmarks):
                gestures.append({
                    'type': 'clap',
                    'confidence': 0.7,
                    'source': 'pose',
                    'timestamp': time.time()
                })
            
        except Exception as e:
            self.get_logger().error(f'Error analyzing pose gestures: {e}')
        
        return gestures
    
    def detect_waving_gesture(self, landmarks: Dict) -> bool:
        """Detect waving gesture based on arm movement"""
        try:
            if 'left_wrist' not in landmarks or 'right_wrist' not in landmarks:
                return False
            
            # Check if one arm is raised and moving
            left_wrist = landmarks['left_wrist']
            right_wrist = landmarks['right_wrist']
            left_shoulder = landmarks.get('left_shoulder', {'y': 0})
            right_shoulder = landmarks.get('right_shoulder', {'y': 0})
            
            # Check if wrist is above shoulder (raised arm)
            left_raised = left_wrist['y'] < left_shoulder['y']
            right_raised = right_wrist['y'] < right_shoulder['y']
            
            # Simple waving detection (can be improved with temporal analysis)
            return left_raised or right_raised
            
        except Exception as e:
            self.get_logger().error(f'Error detecting waving gesture: {e}')
            return False
    
    def detect_pointing_gesture(self, landmarks: Dict) -> bool:
        """Detect pointing gesture based on arm extension"""
        try:
            if 'left_wrist' not in landmarks or 'right_wrist' not in landmarks:
                return False
            
            left_wrist = landmarks['left_wrist']
            right_wrist = landmarks['right_wrist']
            left_elbow = landmarks.get('left_elbow', {'x': 0, 'y': 0})
            right_elbow = landmarks.get('right_elbow', {'x': 0, 'y': 0})
            
            # Check if arm is extended forward
            left_extended = abs(left_wrist['x'] - left_elbow['x']) > 0.1
            right_extended = abs(right_wrist['x'] - right_elbow['x']) > 0.1
            
            return left_extended or right_extended
            
        except Exception as e:
            self.get_logger().error(f'Error detecting pointing gesture: {e}')
            return False
    
    def detect_raised_hands_gesture(self, landmarks: Dict) -> bool:
        """Detect raised hands gesture"""
        try:
            if 'left_wrist' not in landmarks or 'right_wrist' not in landmarks:
                return False
            
            left_wrist = landmarks['left_wrist']
            right_wrist = landmarks['right_wrist']
            left_shoulder = landmarks.get('left_shoulder', {'y': 0})
            right_shoulder = landmarks.get('right_shoulder', {'y': 0})
            
            # Check if both hands are raised above shoulders
            left_raised = left_wrist['y'] < left_shoulder['y']
            right_raised = right_wrist['y'] < right_shoulder['y']
            
            return left_raised and right_raised
            
        except Exception as e:
            self.get_logger().error(f'Error detecting raised hands gesture: {e}')
            return False
    
    def detect_clapping_gesture(self, landmarks: Dict) -> bool:
        """Detect clapping gesture based on hand proximity"""
        try:
            if 'left_wrist' not in landmarks or 'right_wrist' not in landmarks:
                return False
            
            left_wrist = landmarks['left_wrist']
            right_wrist = landmarks['right_wrist']
            
            # Calculate distance between wrists
            distance = math.sqrt(
                (left_wrist['x'] - right_wrist['x'])**2 +
                (left_wrist['y'] - right_wrist['y'])**2
            )
            
            # Check if hands are close together (clapping)
            return distance < 0.1
            
        except Exception as e:
            self.get_logger().error(f'Error detecting clapping gesture: {e}')
            return False
    
    def analyze_basic_gestures(self, gestures: List[str]) -> List[Dict]:
        """Analyze basic gesture strings and convert to detailed format"""
        detailed_gestures = []
        
        for gesture in gestures:
            detailed_gesture = {
                'type': gesture,
                'confidence': 0.5,  # Default confidence for basic gestures
                'source': 'basic',
                'timestamp': time.time()
            }
            detailed_gestures.append(detailed_gesture)
        
        return detailed_gestures
    
    def update_gestures(self, new_gestures: List[Dict], source: str):
        """Update current gestures list"""
        current_time = time.time()
        
        # Add new gestures
        for gesture in new_gestures:
            gesture['source'] = source
            gesture['timestamp'] = current_time
            self.current_gestures.append(gesture)
        
        # Update gesture history
        self.gesture_history.extend(new_gestures)
        
        # Keep only recent gestures in history
        cutoff_time = current_time - self.gesture_timeout
        self.gesture_history = [
            g for g in self.gesture_history
            if g['timestamp'] > cutoff_time
        ]
        
        self.last_gesture_time = current_time
    
    def gesture_analysis_callback(self):
        """Timer callback for gesture analysis"""
        try:
            # Analyze current gestures
            if self.current_gestures:
                self.analyze_gesture_sequences()
                self.calculate_gesture_confidence()
                self.publish_detailed_gestures()
            
        except Exception as e:
            self.get_logger().error(f'Error in gesture analysis: {e}')
    
    def analyze_gesture_sequences(self):
        """Analyze gesture sequences for patterns"""
        try:
            if len(self.gesture_history) < 2:
                return
            
            # Look for gesture patterns
            recent_gestures = self.gesture_history[-self.sequence_length:]
            
            # Analyze for common patterns
            patterns = self.detect_gesture_patterns(recent_gestures)
            
            if patterns:
                # Publish gesture sequences
                sequence_msg = String()
                sequence_data = {
                    'patterns': patterns,
                    'timestamp': time.time(),
                    'gesture_count': len(recent_gestures)
                }
                sequence_msg.data = json.dumps(sequence_data)
                self.gesture_sequences_pub.publish(sequence_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error analyzing gesture sequences: {e}')
    
    def detect_gesture_patterns(self, gestures: List[Dict]) -> List[str]:
        """Detect patterns in gesture sequences"""
        patterns = []
        
        try:
            gesture_types = [g['type'] for g in gestures]
            
            # Check for repeated gestures
            if len(set(gesture_types)) == 1 and len(gesture_types) >= 3:
                patterns.append(f"repeated_{gesture_types[0]}")
            
            # Check for alternating gestures
            if len(gesture_types) >= 4:
                if gesture_types[0] == gesture_types[2] and gesture_types[1] == gesture_types[3]:
                    patterns.append(f"alternating_{gesture_types[0]}_{gesture_types[1]}")
            
            # Check for gesture combinations
            if 'wave' in gesture_types and 'point' in gesture_types:
                patterns.append("wave_and_point")
            
            if 'raised_hands' in gesture_types and 'clap' in gesture_types:
                patterns.append("celebration")
            
        except Exception as e:
            self.get_logger().error(f'Error detecting gesture patterns: {e}')
        
        return patterns
    
    def calculate_gesture_confidence(self):
        """Calculate confidence scores for current gestures"""
        try:
            if not self.current_gestures:
                return
            
            # Calculate confidence based on multiple factors
            for gesture in self.current_gestures:
                base_confidence = gesture.get('confidence', 0.5)
                
                # Adjust confidence based on gesture history
                recent_occurrences = sum(
                    1 for g in self.gesture_history[-10:]
                    if g['type'] == gesture['type']
                )
                
                # Higher confidence for frequently occurring gestures
                if recent_occurrences > 1:
                    base_confidence += 0.1 * min(recent_occurrences, 5)
                
                # Adjust confidence based on source
                if gesture['source'] == 'pose':
                    base_confidence += 0.1
                
                # Cap confidence at 1.0
                gesture['confidence'] = min(base_confidence, 1.0)
            
            # Publish confidence scores
            confidence_msg = String()
            confidence_data = {
                'gestures': [
                    {
                        'type': g['type'],
                        'confidence': g['confidence'],
                        'source': g['source']
                    }
                    for g in self.current_gestures
                ],
                'timestamp': time.time()
            }
            confidence_msg.data = json.dumps(confidence_data)
            self.gesture_confidence_pub.publish(confidence_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error calculating gesture confidence: {e}')
    
    def publish_detailed_gestures(self):
        """Publish detailed gesture analysis"""
        try:
            detailed_msg = String()
            detailed_data = {
                'current_gestures': self.current_gestures,
                'gesture_history': self.gesture_history[-10:],  # Last 10 gestures
                'timestamp': time.time(),
                'total_gestures_detected': len(self.gesture_history)
            }
            detailed_msg.data = json.dumps(detailed_data)
            self.detailed_gestures_pub.publish(detailed_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing detailed gestures: {e}')
    
    def gesture_cleanup_callback(self):
        """Timer callback for cleaning up old gestures"""
        try:
            current_time = time.time()
            
            # Remove old gestures from current list
            self.current_gestures = [
                g for g in self.current_gestures
                if current_time - g['timestamp'] < self.gesture_timeout
            ]
            
            # Clean up gesture history
            cutoff_time = current_time - self.gesture_timeout * 2
            self.gesture_history = [
                g for g in self.gesture_history
                if g['timestamp'] > cutoff_time
            ]
            
        except Exception as e:
            self.get_logger().error(f'Error in gesture cleanup: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = GestureRecognitionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 