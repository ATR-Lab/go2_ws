#!/usr/bin/env python3
"""
Human Detection Node

This node processes camera data from the Go2 robot to detect humans and their gestures.
It uses YOLOv8 for human detection and MediaPipe for pose estimation and gesture recognition.

Subscribes to:
- /camera/image_raw: Camera feed from Go2 robot

Publishes to:
- /human_detection/people: List of detected humans with metadata
- /human_detection/gestures: Recognized hand gestures
- /human_detection/proximity: Distance and zone information
- /human_detection/pose: Body pose and movement data
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
from ultralytics import YOLO
import mediapipe as mp
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_msgs.msg import Header, String
import json
import threading
import time
import os
from typing import List, Dict, Optional, Tuple
import math

# MediaPipe setup
mp_pose = mp.solutions.pose
mp_hands = mp.solutions.hands
mp_face = mp.solutions.face_detection
mp_drawing = mp.solutions.drawing_utils

# Try to import MediaPipe gesture recognizer (newer versions)
try:
    import mediapipe.tasks as mp_tasks
    from mediapipe.tasks.python import vision
    GESTURE_RECOGNIZER_AVAILABLE = True
except ImportError:
    GESTURE_RECOGNIZER_AVAILABLE = False

class HumanDetectionNode(Node):
    """Human detection and gesture recognition node"""
    
    def __init__(self):
        super().__init__('human_detection_node')
        
        # Initialize parameters
        self.setup_parameters()
        
        # Initialize detection models
        self.initialize_models()
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Setup publishers
        self.setup_publishers()
        
        # Setup subscribers
        self.setup_subscribers()
        
        # Initialize state
        self.frame_count = 0
        self.processing_frame = False
        self.camera_info = None
        self.last_detection_time = time.time()
        
        # Threading for processing
        self.processing_lock = threading.Lock()
        
        self.get_logger().info('Human Detection Node initialized')
    
    def setup_parameters(self):
        """Setup node parameters"""
        self.declare_parameter('detection_confidence', 0.6)
        self.declare_parameter('processing_frequency', 10.0)  # Hz
        self.declare_parameter('max_detection_distance', 10.0)  # meters
        self.declare_parameter('interaction_zone_distance', 2.0)  # meters
        self.declare_parameter('approach_zone_distance', 5.0)  # meters
        # Try to find model in workspace models directory first, then package share directory
        workspace_models = os.path.join(os.getcwd(), 'models', 'yolov8n.pt')
        if os.path.exists(workspace_models):
            default_model_path = workspace_models
        else:
            # Fallback to package share directory
            package_share_dir = get_package_share_directory('human_interaction')
            default_model_path = os.path.join(package_share_dir, 'models', 'yolov8n.pt')
        
        self.declare_parameter('model_path', default_model_path)
        self.declare_parameter('enable_pose_estimation', True)
        self.declare_parameter('enable_gesture_recognition', True)
        
        # Get parameters
        self.detection_confidence = self.get_parameter('detection_confidence').value
        self.processing_freq = self.get_parameter('processing_frequency').value
        self.max_distance = self.get_parameter('max_detection_distance').value
        self.interaction_distance = self.get_parameter('interaction_zone_distance').value
        self.approach_distance = self.get_parameter('approach_zone_distance').value
        self.model_path = self.get_parameter('model_path').value
        self.enable_pose = self.get_parameter('enable_pose_estimation').value
        self.enable_gestures = self.get_parameter('enable_gesture_recognition').value
    
    def initialize_models(self):
        """Initialize YOLO and MediaPipe models"""
        try:
            # Load YOLO model
            self.get_logger().info(f'Loading YOLO model from {self.model_path}')
            self.yolo_model = YOLO(self.model_path)
            
            # Initialize MediaPipe
            if self.enable_pose:
                self.pose = mp_pose.Pose(
                    min_detection_confidence=0.5,
                    min_tracking_confidence=0.5
                )
            
            if self.enable_gestures:
                self.hands = mp_hands.Hands(
                    min_detection_confidence=0.7,
                    min_tracking_confidence=0.5,
                    max_num_hands=2
                )
                
                # Initialize gesture history for temporal analysis
                self.gesture_history = []
                self.gesture_history_size = 10
                self.last_gesture_time = time.time()
                self.gesture_cooldown = 0.5  # seconds
            
            self.get_logger().info('Models initialized successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize models: {e}')
            raise
    
    def setup_publishers(self):
        """Setup ROS publishers"""
        # Human detection results
        self.people_pub = self.create_publisher(
            String, 
            '/human_detection/people', 
            10
        )
        
        # Gesture recognition results
        self.gestures_pub = self.create_publisher(
            String, 
            '/human_detection/gestures', 
            10
        )
        
        # Proximity information
        self.proximity_pub = self.create_publisher(
            String, 
            '/human_detection/proximity', 
            10
        )
        
        # Pose data
        self.pose_pub = self.create_publisher(
            PoseArray, 
            '/human_detection/pose', 
            10
        )
    
    def setup_subscribers(self):
        """Setup ROS subscribers"""
        # Camera image
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Camera info
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )
    
    def camera_info_callback(self, msg: CameraInfo):
        """Handle camera calibration info"""
        self.camera_info = msg
        self.get_logger().debug('Received camera info')
    
    def image_callback(self, msg: Image):
        """Process incoming camera images"""
        if self.processing_frame:
            return
        
        self.frame_count += 1
        
        # Process every 3rd frame for performance
        if self.frame_count % 3 != 0:
            return
        
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process frame in separate thread
            threading.Thread(
                target=self.process_frame,
                args=(cv_image, msg.header.stamp),
                daemon=True
            ).start()
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def process_frame(self, frame: np.ndarray, timestamp):
        """Process frame for human detection and gesture recognition"""
        with self.processing_lock:
            self.processing_frame = True
            
            try:
                # Detect humans with YOLO
                human_detections = self.detect_humans(frame)
                
                # Process each detected human
                for detection in human_detections:
                    # Extract human bounding box
                    bbox = detection['bbox']
                    confidence = detection['confidence']
                    
                    # Crop human region for detailed analysis
                    human_region = self.crop_region(frame, bbox)
                    
                    # Pose estimation
                    pose_data = None
                    if self.enable_pose and human_region is not None:
                        pose_data = self.estimate_pose(human_region)
                    
                    # Gesture recognition
                    gestures = None
                    if self.enable_gestures and human_region is not None:
                        gestures = self.recognize_gestures(human_region)
                    
                    # Calculate proximity
                    distance = self.calculate_distance(bbox, confidence)
                    zone = self.classify_zone(distance)
                    
                    # Publish results
                    self.publish_detection_results(
                        detection, pose_data, gestures, distance, zone, timestamp
                    )
                
                self.last_detection_time = time.time()
                
            except Exception as e:
                self.get_logger().error(f'Error in frame processing: {e}')
            
            finally:
                self.processing_frame = False
    
    def detect_humans(self, frame: np.ndarray) -> List[Dict]:
        """Detect humans using YOLOv8"""
        try:
            # Run YOLO detection
            results = self.yolo_model(frame, verbose=False)
            
            human_detections = []
            
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # Check if detection is human (class 0 in COCO dataset)
                        if box.cls == 0:  # Human class
                            confidence = float(box.conf[0])
                            
                            if confidence >= self.detection_confidence:
                                # Get bounding box coordinates
                                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                                
                                detection = {
                                    'bbox': [int(x1), int(y1), int(x2), int(y2)],
                                    'confidence': confidence,
                                    'class': 'human'
                                }
                                
                                human_detections.append(detection)
            
            return human_detections
            
        except Exception as e:
            self.get_logger().error(f'Error in human detection: {e}')
            return []
    
    def estimate_pose(self, frame: np.ndarray) -> Optional[Dict]:
        """Estimate human pose using MediaPipe"""
        try:
            # Convert BGR to RGB
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Process pose estimation
            results = self.pose.process(rgb_frame)
            
            if results.pose_landmarks:
                landmarks = results.pose_landmarks.landmark
                
                # Extract key landmarks
                pose_data = {
                    'nose': [landmarks[mp_pose.PoseLandmark.NOSE].x, 
                            landmarks[mp_pose.PoseLandmark.NOSE].y],
                    'left_shoulder': [landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].x,
                                     landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER].y],
                    'right_shoulder': [landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].x,
                                      landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER].y],
                    'left_elbow': [landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].x,
                                  landmarks[mp_pose.PoseLandmark.LEFT_ELBOW].y],
                    'right_elbow': [landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].x,
                                   landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW].y],
                    'left_wrist': [landmarks[mp_pose.PoseLandmark.LEFT_WRIST].x,
                                  landmarks[mp_pose.PoseLandmark.LEFT_WRIST].y],
                    'right_wrist': [landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].x,
                                   landmarks[mp_pose.PoseLandmark.RIGHT_WRIST].y],
                }
                
                return pose_data
            
            return None
            
        except Exception as e:
            self.get_logger().error(f'Error in pose estimation: {e}')
            return None
    
    def recognize_gestures(self, frame: np.ndarray) -> List[str]:
        """Recognize hand gestures using MediaPipe"""
        try:
            # Convert BGR to RGB
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Process hand detection
            results = self.hands.process(rgb_frame)
            
            gestures = []
            current_time = time.time()
            
            if results.multi_hand_landmarks and results.multi_handedness:
                for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                    # Get hand label (Left/Right)
                    hand_label = handedness.classification[0].label
                    
                    # Analyze hand landmarks for gestures
                    gesture = self.analyze_hand_gesture(hand_landmarks, hand_label)
                    if gesture and gesture != "unknown":
                        gestures.append(f"{hand_label.lower()}_{gesture}")
            
            # Update gesture history for temporal analysis
            if gestures:
                self.gesture_history.append((current_time, gestures))
                
                # Keep only recent history
                cutoff_time = current_time - 2.0  # Keep 2 seconds of history
                self.gesture_history = [(t, g) for t, g in self.gesture_history if t > cutoff_time]
                
                # Detect gesture sequences
                sequence_gestures = self.detect_gesture_sequences()
                gestures.extend(sequence_gestures)
            
            # If no gestures detected, check if hands are present
            if not gestures and results.multi_hand_landmarks:
                gestures = ["hands_visible"]
            
            return list(set(gestures))  # Remove duplicates
            
        except Exception as e:
            self.get_logger().error(f'Error in gesture recognition: {e}')
            return []
    
    def analyze_hand_gesture(self, landmarks, hand_label: str) -> Optional[str]:
        """Analyze hand landmarks to determine specific gesture"""
        try:
            # Extract key hand landmarks
            wrist = landmarks.landmark[mp_hands.HandLandmark.WRIST]
            thumb_tip = landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP]
            thumb_ip = landmarks.landmark[mp_hands.HandLandmark.THUMB_IP]
            thumb_mcp = landmarks.landmark[mp_hands.HandLandmark.THUMB_MCP]
            
            index_tip = landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
            index_pip = landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_PIP]
            index_mcp = landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_MCP]
            
            middle_tip = landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
            middle_pip = landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_PIP]
            middle_mcp = landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_MCP]
            
            ring_tip = landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP]
            ring_pip = landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_PIP]
            ring_mcp = landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_MCP]
            
            pinky_tip = landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP]
            pinky_pip = landmarks.landmark[mp_hands.HandLandmark.PINKY_PIP]
            pinky_mcp = landmarks.landmark[mp_hands.HandLandmark.PINKY_MCP]
            
            # Helper function to check if finger is extended
            def is_finger_extended(tip, pip, mcp):
                return tip.y < pip.y < mcp.y
            
            def is_thumb_extended(tip, ip, mcp, hand_label):
                if hand_label == "Right":
                    return tip.x > ip.x > mcp.x
                else:  # Left hand
                    return tip.x < ip.x < mcp.x
            
            # Check finger states
            thumb_up = is_thumb_extended(thumb_tip, thumb_ip, thumb_mcp, hand_label)
            index_up = is_finger_extended(index_tip, index_pip, index_mcp)
            middle_up = is_finger_extended(middle_tip, middle_pip, middle_mcp)
            ring_up = is_finger_extended(ring_tip, ring_pip, ring_mcp)
            pinky_up = is_finger_extended(pinky_tip, pinky_pip, pinky_mcp)
            
            fingers_up = [thumb_up, index_up, middle_up, ring_up, pinky_up]
            fingers_count = sum(fingers_up)
            
            # Gesture recognition logic
            
            # Thumbs up: Only thumb extended
            if thumb_up and not any([index_up, middle_up, ring_up, pinky_up]):
                return "thumbs_up"
            
            # Pointing: Only index finger extended
            if index_up and not any([thumb_up, middle_up, ring_up, pinky_up]):
                return "pointing"
            
            # Peace sign: Index and middle fingers extended
            if index_up and middle_up and not any([thumb_up, ring_up, pinky_up]):
                return "peace_sign"
            
            # Rock sign: Index and pinky extended
            if index_up and pinky_up and not any([thumb_up, middle_up, ring_up]):
                return "rock_sign"
            
            # Open hand: All fingers extended
            if fingers_count >= 4:
                return "open_hand"
            
            # Fist: No fingers extended
            if fingers_count == 0:
                return "fist"
            
            # OK sign: Thumb and index forming circle (approximate)
            thumb_index_distance = math.sqrt(
                (thumb_tip.x - index_tip.x)**2 + (thumb_tip.y - index_tip.y)**2
            )
            if thumb_index_distance < 0.05 and middle_up and ring_up and pinky_up:
                return "ok_sign"
            
            # Wave detection (based on hand position relative to wrist)
            if index_up and middle_up and ring_up and pinky_up:
                hand_center_y = (index_tip.y + middle_tip.y + ring_tip.y + pinky_tip.y) / 4
                if hand_center_y < wrist.y - 0.1:  # Hand raised above wrist
                    return "wave"
            
            # Default: Unknown gesture
            return "unknown"
            
        except Exception as e:
            self.get_logger().error(f'Error analyzing hand gesture: {e}')
            return None
    
    def detect_gesture_sequences(self) -> List[str]:
        """Detect gesture sequences from history"""
        try:
            if len(self.gesture_history) < 3:
                return []
            
            sequences = []
            
            # Get recent gestures (last 1 second)
            current_time = time.time()
            recent_gestures = []
            for timestamp, gestures in self.gesture_history:
                if current_time - timestamp <= 1.0:
                    recent_gestures.extend(gestures)
            
            # Detect repeated gestures
            gesture_counts = {}
            for gesture in recent_gestures:
                gesture_counts[gesture] = gesture_counts.get(gesture, 0) + 1
            
            for gesture, count in gesture_counts.items():
                if count >= 3:  # Repeated 3+ times
                    sequences.append(f"repeated_{gesture}")
            
            # Detect specific sequences
            if len(recent_gestures) >= 2:
                # Wave followed by pointing
                if any("wave" in g for g in recent_gestures[-3:]) and any("pointing" in g for g in recent_gestures[-2:]):
                    sequences.append("wave_then_point")
                
                # Thumbs up followed by peace sign
                if any("thumbs_up" in g for g in recent_gestures[-3:]) and any("peace_sign" in g for g in recent_gestures[-2:]):
                    sequences.append("thumbs_up_then_peace")
            
            return sequences
            
        except Exception as e:
            self.get_logger().error(f'Error detecting gesture sequences: {e}')
            return []
    
    def calculate_distance(self, bbox: List[int], confidence: float) -> float:
        """Calculate approximate distance to human"""
        try:
            # Simple distance estimation based on bounding box size
            # This is a rough approximation - can be improved with camera calibration
            
            x1, y1, x2, y2 = bbox
            bbox_width = x2 - x1
            bbox_height = y2 - y1
            bbox_area = bbox_width * bbox_height
            
            # Assume a person at 2m distance has a certain bbox area
            # This is a rough calibration that should be tuned
            reference_area = 50000  # pixels at 2m distance
            reference_distance = 2.0  # meters
            
            # Calculate distance using inverse square law
            if bbox_area > 0:
                distance = reference_distance * math.sqrt(reference_area / bbox_area)
                return min(distance, self.max_distance)
            
            return self.max_distance
            
        except Exception as e:
            self.get_logger().error(f'Error calculating distance: {e}')
            return self.max_distance
    
    def classify_zone(self, distance: float) -> str:
        """Classify human into interaction zones"""
        if distance <= self.interaction_distance:
            return "interaction"
        elif distance <= self.approach_distance:
            return "approach"
        else:
            return "far"
    
    def crop_region(self, frame: np.ndarray, bbox: List[int]) -> Optional[np.ndarray]:
        """Crop region from frame"""
        try:
            x1, y1, x2, y2 = bbox
            
            # Ensure coordinates are within frame bounds
            height, width = frame.shape[:2]
            x1 = max(0, min(x1, width))
            y1 = max(0, min(y1, height))
            x2 = max(0, min(x2, width))
            y2 = max(0, min(y2, height))
            
            if x2 > x1 and y2 > y1:
                return frame[y1:y2, x1:x2]
            
            return None
            
        except Exception as e:
            self.get_logger().error(f'Error cropping region: {e}')
            return None
    
    def publish_detection_results(self, detection: Dict, pose_data: Optional[Dict], 
                                gestures: List[str], distance: float, zone: str, timestamp):
        """Publish detection results to ROS topics"""
        try:
            # Publish human detection
            people_msg = String()
            people_data = {
                'timestamp': timestamp.sec + timestamp.nanosec * 1e-9,
                'detection': detection,
                'distance': distance,
                'zone': zone
            }
            people_msg.data = json.dumps(people_data)
            self.people_pub.publish(people_msg)
            
            # Publish gestures
            if gestures:
                gestures_msg = String()
                gestures_data = {
                    'timestamp': timestamp.sec + timestamp.nanosec * 1e-9,
                    'gestures': gestures,
                    'detection_id': detection.get('id', 0)
                }
                gestures_msg.data = json.dumps(gestures_data)
                self.gestures_pub.publish(gestures_msg)
            
            # Publish proximity information
            proximity_msg = String()
            proximity_data = {
                'timestamp': timestamp.sec + timestamp.nanosec * 1e-9,
                'distance': distance,
                'zone': zone,
                'confidence': detection['confidence']
            }
            proximity_msg.data = json.dumps(proximity_data)
            self.proximity_pub.publish(proximity_msg)
            
            # Publish pose data
            if pose_data:
                pose_msg = PoseArray()
                pose_msg.header.stamp = timestamp
                pose_msg.header.frame_id = "camera_frame"
                
                # Convert pose landmarks to ROS Pose messages
                for landmark_name, coords in pose_data.items():
                    pose = Pose()
                    pose.position.x = coords[0]
                    pose.position.y = coords[1]
                    pose.position.z = 0.0
                    pose.orientation.w = 1.0
                    pose_msg.poses.append(pose)
                
                self.pose_pub.publish(pose_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing detection results: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    node = HumanDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 