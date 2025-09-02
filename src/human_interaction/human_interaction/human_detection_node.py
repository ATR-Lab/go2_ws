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
from enum import Enum

class GestureState(Enum):
    """Gesture state enumeration"""
    NEW = "new"
    ONGOING = "ongoing" 
    ENDED = "ended"

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
        
        # Human tracking state
        self.tracked_humans = {}  # Dict[human_id, human_data]
        self.next_human_id = 1
        self.max_tracking_distance = 100  # pixels
        self.human_timeout = 3.0  # seconds
        
        # Gesture state management
        self.gesture_history = []  # List of (timestamp, gestures) tuples
        self.last_gesture_responses = {}  # Dict[human_id, Dict[gesture, timestamp]]
        self.gesture_debounce_time = 2.0  # seconds
        self.gesture_rate_limit = 1.0  # max 1 response per gesture per second
        
        # Gesture state tracking
        self.gesture_states = {}  # Dict[human_id, Dict[gesture, GestureState]]
        self.gesture_start_times = {}  # Dict[human_id, Dict[gesture, timestamp]]
        self.gesture_end_timeout = 1.0  # seconds without gesture = ended
        
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
        
        # Combined gestures for interaction manager
        self.combined_gestures_pub = self.create_publisher(
            String,
            '/human_detection/combined_gestures',
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
                all_gesture_results = []
                
                for detection in human_detections:
                    # Extract human data
                    bbox = detection['bbox']
                    confidence = detection['confidence']
                    human_id = detection['id']
                    
                    # Crop human region for detailed analysis
                    human_region = self.crop_region(frame, bbox)
                    
                    # Pose estimation
                    pose_data = None
                    if self.enable_pose and human_region is not None:
                        pose_data = self.estimate_pose(human_region)
                    
                    # Gesture recognition (now with human attribution)
                    gesture_result = None
                    if self.enable_gestures:
                        # Use full frame for gesture recognition to capture hands properly
                        gesture_result = self.recognize_gestures(frame, human_id, bbox)
                        all_gesture_results.append(gesture_result)
                    
                    # Calculate proximity
                    distance = self.calculate_distance(bbox, confidence)
                    zone = self.classify_zone(distance)
                    
                    # Publish results for this human
                    self.publish_detection_results(
                        detection, pose_data, gesture_result, distance, zone, timestamp
                    )
                
                # Publish combined gesture results for interaction manager
                if all_gesture_results:
                    self.publish_combined_gestures(all_gesture_results, timestamp)
                
                self.last_detection_time = time.time()
                
            except Exception as e:
                self.get_logger().error(f'Error in frame processing: {e}')
            
            finally:
                self.processing_frame = False
    
    def detect_humans(self, frame: np.ndarray) -> List[Dict]:
        """Detect humans using YOLOv8 with tracking"""
        try:
            # Run YOLO detection
            results = self.yolo_model(frame, verbose=False)
            
            raw_detections = []
            
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
                                    'class': 'human',
                                    'center': [int((x1 + x2) / 2), int((y1 + y2) / 2)],
                                    'area': int((x2 - x1) * (y2 - y1))
                                }
                                
                                raw_detections.append(detection)
            
            # Apply human tracking to assign IDs
            tracked_detections = self.track_humans(raw_detections)
            
            return tracked_detections
            
        except Exception as e:
            self.get_logger().error(f'Error in human detection: {e}')
            return []
    
    def track_humans(self, detections: List[Dict]) -> List[Dict]:
        """Track humans across frames and assign consistent IDs"""
        current_time = time.time()
        
        # Clean up old tracked humans
        expired_ids = []
        for human_id, human_data in self.tracked_humans.items():
            if current_time - human_data['last_seen'] > self.human_timeout:
                expired_ids.append(human_id)
        
        for human_id in expired_ids:
            del self.tracked_humans[human_id]
            # Clean up gesture history for this human
            if human_id in self.last_gesture_responses:
                del self.last_gesture_responses[human_id]
            # Clean up gesture states
            if human_id in self.gesture_states:
                del self.gesture_states[human_id]
            if human_id in self.gesture_start_times:
                del self.gesture_start_times[human_id]
        
        tracked_detections = []
        
        for detection in detections:
            detection_center = detection['center']
            detection_area = detection['area']
            
            # Find best matching tracked human
            best_match_id = None
            best_distance = float('inf')
            
            for human_id, human_data in self.tracked_humans.items():
                # Calculate distance between current detection and tracked human
                tracked_center = human_data['center']
                distance = math.sqrt(
                    (detection_center[0] - tracked_center[0])**2 + 
                    (detection_center[1] - tracked_center[1])**2
                )
                
                # Consider area similarity as well
                area_ratio = min(detection_area, human_data['area']) / max(detection_area, human_data['area'])
                
                # Combined matching score (distance + area similarity)
                if distance < self.max_tracking_distance and area_ratio > 0.5:
                    if distance < best_distance:
                        best_distance = distance
                        best_match_id = human_id
            
            if best_match_id is not None:
                # Update existing tracked human
                detection['id'] = best_match_id
                self.tracked_humans[best_match_id].update({
                    'center': detection_center,
                    'area': detection_area,
                    'bbox': detection['bbox'],
                    'confidence': detection['confidence'],
                    'last_seen': current_time
                })
            else:
                # Create new tracked human
                detection['id'] = self.next_human_id
                self.tracked_humans[self.next_human_id] = {
                    'center': detection_center,
                    'area': detection_area,
                    'bbox': detection['bbox'],
                    'confidence': detection['confidence'],
                    'last_seen': current_time,
                    'first_seen': current_time
                }
                self.next_human_id += 1
            
            tracked_detections.append(detection)
        
        return tracked_detections
    
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
    
    def recognize_gestures(self, frame: np.ndarray, human_id: int, human_bbox: List[int]) -> Dict:
        """Recognize hand gestures using MediaPipe with human attribution"""
        try:
            # Convert BGR to RGB
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Process hand detection
            results = self.hands.process(rgb_frame)
            
            gestures = []
            current_time = time.time()
            
            if results.multi_hand_landmarks and results.multi_handedness:
                for hand_landmarks, handedness in zip(results.multi_hand_landmarks, results.multi_handedness):
                    # Check if hand is within human bounding box
                    if self.is_hand_in_human_bbox(hand_landmarks, human_bbox, rgb_frame.shape):
                        # Get hand label (Left/Right)
                        hand_label = handedness.classification[0].label
                        
                        # Analyze hand landmarks for gestures
                        gesture = self.analyze_hand_gesture(hand_landmarks, hand_label)
                        if gesture and gesture != "unknown":
                            full_gesture = f"{hand_label.lower()}_{gesture}"
                            
                            # Check if this gesture should be debounced
                            if self.should_respond_to_gesture(human_id, full_gesture, current_time):
                                gestures.append(full_gesture)
            
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
                # Only add hands_visible if hands are in this human's bbox
                for hand_landmarks in results.multi_hand_landmarks:
                    if self.is_hand_in_human_bbox(hand_landmarks, human_bbox, rgb_frame.shape):
                        gestures = ["hands_visible"]
                        break
            
            # Update gesture states
            gesture_states = self.update_gesture_states(human_id, gestures, current_time)
            
            return {
                'human_id': human_id,
                'gestures': list(set(gestures)),  # Remove duplicates
                'gesture_states': gesture_states,
                'timestamp': current_time
            }
            
        except Exception as e:
            self.get_logger().error(f'Error in gesture recognition: {e}')
            return {'human_id': human_id, 'gestures': [], 'timestamp': current_time}
    
    def is_hand_in_human_bbox(self, hand_landmarks, human_bbox: List[int], frame_shape: Tuple) -> bool:
        """Check if hand landmarks are within human bounding box"""
        try:
            height, width = frame_shape[:2]
            x1, y1, x2, y2 = human_bbox
            
            # Get hand center from landmarks
            hand_x = sum([lm.x for lm in hand_landmarks.landmark]) / len(hand_landmarks.landmark)
            hand_y = sum([lm.y for lm in hand_landmarks.landmark]) / len(hand_landmarks.landmark)
            
            # Convert normalized coordinates to pixel coordinates
            hand_x_px = int(hand_x * width)
            hand_y_px = int(hand_y * height)
            
            # Check if hand center is within human bounding box (with some margin)
            margin = 50  # pixels
            return (x1 - margin <= hand_x_px <= x2 + margin and 
                    y1 - margin <= hand_y_px <= y2 + margin)
                    
        except Exception as e:
            self.get_logger().error(f'Error checking hand-human association: {e}')
            return False
    
    def should_respond_to_gesture(self, human_id: int, gesture: str, current_time: float) -> bool:
        """Check if we should respond to this gesture (debouncing)"""
        try:
            # Initialize gesture history for this human if needed
            if human_id not in self.last_gesture_responses:
                self.last_gesture_responses[human_id] = {}
            
            # Check if we've responded to this gesture recently
            if gesture in self.last_gesture_responses[human_id]:
                last_response_time = self.last_gesture_responses[human_id][gesture]
                if current_time - last_response_time < self.gesture_debounce_time:
                    return False  # Too soon, debounce this gesture
            
            # Record this gesture response
            self.last_gesture_responses[human_id][gesture] = current_time
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error in gesture debouncing: {e}')
            return True  # Default to allowing gesture
    
    def update_gesture_states(self, human_id: int, current_gestures: List[str], current_time: float) -> Dict:
        """Update gesture states for a human (NEW → ONGOING → ENDED)"""
        try:
            # Initialize state tracking for this human if needed
            if human_id not in self.gesture_states:
                self.gesture_states[human_id] = {}
                self.gesture_start_times[human_id] = {}
            
            human_gesture_states = self.gesture_states[human_id]
            human_start_times = self.gesture_start_times[human_id]
            
            # Track current gesture states
            current_states = {}
            
            # Process current gestures
            for gesture in current_gestures:
                if gesture in human_gesture_states:
                    # Gesture was already active - mark as ONGOING
                    if human_gesture_states[gesture] == GestureState.NEW:
                        human_gesture_states[gesture] = GestureState.ONGOING
                        current_states[gesture] = GestureState.ONGOING
                    else:
                        current_states[gesture] = GestureState.ONGOING
                else:
                    # New gesture detected - mark as NEW
                    human_gesture_states[gesture] = GestureState.NEW
                    human_start_times[gesture] = current_time
                    current_states[gesture] = GestureState.NEW
                    self.get_logger().debug(f'NEW gesture "{gesture}" detected for human {human_id}')
            
            # Check for ended gestures (gestures that were active but not detected now)
            ended_gestures = []
            for gesture, state in list(human_gesture_states.items()):
                if gesture not in current_gestures and state != GestureState.ENDED:
                    # Gesture has ended
                    human_gesture_states[gesture] = GestureState.ENDED
                    current_states[gesture] = GestureState.ENDED
                    ended_gestures.append(gesture)
                    self.get_logger().debug(f'ENDED gesture "{gesture}" for human {human_id}')
            
            # Clean up old ended gestures (older than timeout)
            gestures_to_remove = []
            for gesture, state in human_gesture_states.items():
                if (state == GestureState.ENDED and 
                    gesture in human_start_times and
                    current_time - human_start_times[gesture] > self.gesture_end_timeout * 3):
                    gestures_to_remove.append(gesture)
            
            for gesture in gestures_to_remove:
                del human_gesture_states[gesture]
                if gesture in human_start_times:
                    del human_start_times[gesture]
            
            return current_states
            
        except Exception as e:
            self.get_logger().error(f'Error updating gesture states: {e}')
            return {}
    
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
                                gesture_result: Optional[Dict], distance: float, zone: str, timestamp):
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
            
            # Publish individual gestures for this human
            if gesture_result and gesture_result.get('gestures'):
                gestures_msg = String()
                gestures_data = {
                    'timestamp': gesture_result.get('timestamp', timestamp.sec + timestamp.nanosec * 1e-9),
                    'gestures': gesture_result['gestures'],
                    'human_id': gesture_result['human_id']
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
    
    def publish_combined_gestures(self, all_gesture_results: List[Dict], timestamp):
        """Publish combined gesture results for interaction manager"""
        try:
            # Create a priority-ordered list of gestures
            prioritized_gestures = self.prioritize_gestures(all_gesture_results)
            
            if prioritized_gestures:
                combined_msg = String()
                combined_data = {
                    'timestamp': timestamp.sec + timestamp.nanosec * 1e-9,
                    'all_humans': all_gesture_results,
                    'prioritized_gestures': prioritized_gestures,
                    'total_humans': len(all_gesture_results)
                }
                combined_msg.data = json.dumps(combined_data)
                # Publish to a new topic for the interaction manager
                # We'll need to add this publisher in setup_publishers
                if hasattr(self, 'combined_gestures_pub'):
                    self.combined_gestures_pub.publish(combined_msg)
                    
        except Exception as e:
            self.get_logger().error(f'Error publishing combined gestures: {e}')
    
    def prioritize_gestures(self, all_gesture_results: List[Dict]) -> List[Dict]:
        """Prioritize gestures based on human proximity and gesture importance"""
        try:
            prioritized = []
            
            for gesture_result in all_gesture_results:
                if gesture_result.get('gestures'):
                    human_id = gesture_result['human_id']
                    
                    # Get human data for prioritization
                    human_data = self.tracked_humans.get(human_id, {})
                    
                    # Calculate priority score (closer humans get higher priority)
                    bbox = human_data.get('bbox', [0, 0, 100, 100])
                    area = human_data.get('area', 1)
                    confidence = human_data.get('confidence', 0.0)
                    
                    # Larger area (closer) and higher confidence = higher priority
                    priority_score = area * confidence
                    
                    prioritized.append({
                        'human_id': human_id,
                        'gestures': gesture_result['gestures'],
                        'priority_score': priority_score,
                        'timestamp': gesture_result['timestamp']
                    })
            
            # Sort by priority score (highest first)
            prioritized.sort(key=lambda x: x['priority_score'], reverse=True)
            
            return prioritized
            
        except Exception as e:
            self.get_logger().error(f'Error prioritizing gestures: {e}')
            return []


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