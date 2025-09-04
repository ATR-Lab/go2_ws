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
from boxmot import ByteTrack

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
    from mediapipe.tasks.python.vision import GestureRecognizer, GestureRecognizerOptions
    from mediapipe.tasks.python.components import processors
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
        
        # Dynamic FPS measurement for adaptive gesture recognition
        self.frame_timestamps = []
        self.measured_fps = 10.0  # Default assumption
        self.fps_measurement_window = 30  # frames
        
        # ByteTrack initialization for robust human tracking
        self.tracker = ByteTrack(
            track_thresh=0.6,      # Higher threshold for laggy video
            track_buffer=60,       # Longer buffer for frame gaps (frames)
            match_thresh=0.8,      # More permissive matching for WebRTC
            frame_rate=10          # Match our actual camera FPS
        )
        
        # Human tracking state (simplified for ByteTrack)
        self.tracked_humans = {}  # Dict[track_id, human_metadata]
        self.human_timeout = 3.0  # seconds
        
        # Gesture state management - RELAXED FOR BETTER RESPONSIVENESS
        self.gesture_history = []  # List of (timestamp, gestures) tuples
        self.last_gesture_responses = {}  # Dict[human_id, Dict[gesture, timestamp]]
        self.gesture_debounce_time = 2.0  # seconds - reduced from 5.0 for faster detection
        self.gesture_rate_limit = 1.0  # max 1 response per gesture per second
        
        # Gesture stability tracking - RELAXED REQUIREMENTS
        self.gesture_stability_buffer = {}  # Dict[human_id, Dict[gesture, List[timestamps]]]
        self.gesture_stability_window = 1.0  # seconds - reduced from 2.0 for faster response
        self.gesture_stability_threshold = 0.2  # 20% of frames must show same gesture (reduced from 30%)
        self.min_gesture_confidence_frames = 1  # minimum frames to confirm gesture
        
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
                
                # Initialize MediaPipe Gesture Recognizer (primary method)
                self.gesture_recognizer = None
                if GESTURE_RECOGNIZER_AVAILABLE:
                    try:
                        # Try to find MediaPipe gesture recognizer model
                        mediapipe_model_path = os.path.join(os.getcwd(), 'models', 'mediapipe', 'gesture_recognizer.task')
                        if not os.path.exists(mediapipe_model_path):
                            # Fallback to workspace models directory
                            mediapipe_model_path = os.path.join(os.getcwd(), 'models', 'gesture_recognizer.task')
                        
                        if os.path.exists(mediapipe_model_path):
                            options = GestureRecognizerOptions(
                                base_options=mp_tasks.BaseOptions(
                                    model_asset_path=mediapipe_model_path
                                ),
                                running_mode=vision.RunningMode.IMAGE,
                                num_hands=2,
                                min_hand_detection_confidence=0.7,
                                min_hand_presence_confidence=0.5,
                                min_tracking_confidence=0.5
                            )
                            self.gesture_recognizer = GestureRecognizer.create_from_options(options)
                            self.get_logger().info(f'MediaPipe Gesture Recognizer initialized successfully (PRIMARY METHOD) - Model: {mediapipe_model_path}')
                        else:
                            self.get_logger().warn(f'MediaPipe gesture recognizer model not found at {mediapipe_model_path}')
                            self.gesture_recognizer = None
                    except Exception as e:
                        self.get_logger().warn(f'Failed to initialize MediaPipe Gesture Recognizer: {e}')
                        self.gesture_recognizer = None
                
                if self.gesture_recognizer is None:
                    self.get_logger().info('Using custom gesture detection as fallback (FALLBACK METHOD)')
                
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
        # Camera image with Go2-compatible QoS
        camera_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            camera_qos
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
        current_time = time.time()
        
        # Update FPS measurement
        self.update_fps_measurement(current_time)
        
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
            
                            # Apply ByteTrack for robust human tracking
                tracked_detections = self.track_humans_with_bytetrack(raw_detections, frame)
            
            return tracked_detections
            
        except Exception as e:
            self.get_logger().error(f'Error in human detection: {e}')
            return []
    
    def track_humans_with_bytetrack(self, detections: List[Dict], frame: np.ndarray) -> List[Dict]:
        """Use ByteTrack for robust human tracking across frames"""
        current_time = time.time()
        
        if not detections:
            return []
        
        # Convert our detection format to ByteTrack format
        # ByteTrack expects: [x1, y1, x2, y2, conf, class_id]
        bytetrack_dets = []
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            conf = det['confidence']
            class_id = 0  # Human class
            bytetrack_dets.append([x1, y1, x2, y2, conf, class_id])
        
        # Convert to numpy array
        dets_array = np.array(bytetrack_dets) if bytetrack_dets else np.empty((0, 6))
        
        # Update ByteTrack tracker
        # Returns: [x1, y1, x2, y2, track_id, conf, class_id, det_ind]
        tracks = self.tracker.update(dets_array, frame)
        
        # Convert ByteTrack results back to our format
        tracked_detections = []
        
        for track in tracks:
            if len(track) >= 5:  # Ensure we have at least x1, y1, x2, y2, track_id
                x1, y1, x2, y2 = track[:4]
                track_id = int(track[4])
                conf = track[5] if len(track) > 5 else 0.8
                
                # Create detection in our format
                detection = {
                    'id': track_id,
                    'bbox': [int(x1), int(y1), int(x2), int(y2)],
                    'confidence': float(conf),
                    'class': 'human',
                    'center': [int((x1 + x2) / 2), int((y1 + y2) / 2)],
                    'area': int((x2 - x1) * (y2 - y1))
                }
                
                # Update our tracking metadata
                self.tracked_humans[track_id] = {
                    'center': detection['center'],
                    'area': detection['area'],
                    'bbox': detection['bbox'],
                    'confidence': detection['confidence'],
                    'last_seen': current_time,
                    'first_seen': self.tracked_humans.get(track_id, {}).get('first_seen', current_time)
                }
                
                tracked_detections.append(detection)
        
        # Clean up old tracking metadata
        current_track_ids = {det['id'] for det in tracked_detections}
        expired_ids = []
        for track_id, human_data in self.tracked_humans.items():
            if (track_id not in current_track_ids and 
                current_time - human_data['last_seen'] > self.human_timeout):
                expired_ids.append(track_id)
        
        for track_id in expired_ids:
            del self.tracked_humans[track_id]
            # Clean up gesture history for this human
            if track_id in self.last_gesture_responses:
                del self.last_gesture_responses[track_id]
            if track_id in self.gesture_states:
                del self.gesture_states[track_id]
            if track_id in self.gesture_start_times:
                del self.gesture_start_times[track_id]
        
        return tracked_detections
    
    def update_fps_measurement(self, current_time: float):
        """Update measured FPS for adaptive gesture recognition"""
        try:
            # Add current timestamp
            self.frame_timestamps.append(current_time)
            
            # Keep only recent timestamps
            if len(self.frame_timestamps) > self.fps_measurement_window:
                self.frame_timestamps.pop(0)
            
            # Calculate FPS if we have enough samples
            if len(self.frame_timestamps) >= 10:
                time_span = self.frame_timestamps[-1] - self.frame_timestamps[0]
                if time_span > 0:
                    measured_fps = (len(self.frame_timestamps) - 1) / time_span
                    # Smooth the FPS measurement
                    self.measured_fps = 0.8 * self.measured_fps + 0.2 * measured_fps
                    
        except Exception as e:
            self.get_logger().error(f'Error updating FPS measurement: {e}')
    
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
        """Recognize hand gestures using MediaPipe Gesture Recognizer only"""
        try:
            # Convert BGR to RGB
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            current_time = time.time()
            gestures = []
            detection_method = "unavailable"
            
            # Use only MediaPipe Gesture Recognizer
            if self.gesture_recognizer is not None:
                gestures, detection_method = self.recognize_gestures_mediapipe(rgb_frame, human_id, human_bbox, current_time)
            else:
                self.get_logger().warn('MediaPipe Gesture Recognizer not available')
            
            # Update gesture states
            gesture_states = self.update_gesture_states(human_id, gestures, current_time)
            
            return {
                'human_id': human_id,
                'gestures': list(set(gestures)),  # Remove duplicates
                'gesture_states': gesture_states,
                'timestamp': current_time,
                'detection_method': detection_method
            }
            
        except Exception as e:
            self.get_logger().error(f'Error in gesture recognition: {e}')
            return {'human_id': human_id, 'gestures': [], 'timestamp': current_time, 'detection_method': 'error'}
    
    def recognize_gestures_mediapipe(self, rgb_frame: np.ndarray, human_id: int, human_bbox: List[int], current_time: float) -> Tuple[List[str], str]:
        """Use MediaPipe Gesture Recognizer for gesture detection"""
        try:
            # Create MediaPipe Image
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
            
            # Run gesture recognition
            gesture_recognition_result = self.gesture_recognizer.recognize(mp_image)
            
            gestures = []
            
            if gesture_recognition_result.gestures:
                for i, gesture_list in enumerate(gesture_recognition_result.gestures):
                    if gesture_list and len(gesture_recognition_result.handedness) > i:
                        # Get the top gesture
                        top_gesture = gesture_list[0]
                        handedness = gesture_recognition_result.handedness[i][0]
                        
                        # Check if hand is within human bounding box
                        if (len(gesture_recognition_result.hand_landmarks) > i and 
                            self.is_hand_in_human_bbox_mp(gesture_recognition_result.hand_landmarks[i], human_bbox, rgb_frame.shape)):
                            
                            hand_label = handedness.category_name.lower()  # "left" or "right"
                            gesture_name = top_gesture.category_name.lower()
                            confidence = top_gesture.score
                            
                            # Only accept gestures with reasonable confidence
                            if confidence > 0.6:  # Slightly lower threshold for MediaPipe
                                # Map MediaPipe gesture names to our format
                                gesture_mapping = {
                                    'thumb_up': 'thumbs_up',
                                    'thumb_down': 'thumbs_down', 
                                    'victory': 'peace_sign',
                                    'pointing_up': 'pointing',
                                    'closed_fist': 'fist',
                                    'open_palm': 'open_hand',
                                    'iloveyou': 'rock_sign'
                                }
                                mapped_gesture = gesture_mapping.get(gesture_name, gesture_name)
                                full_gesture = f"{hand_label}_{mapped_gesture}"
                                
                                self.get_logger().debug(f'MediaPipe detected gesture "{gesture_name}" -> "{mapped_gesture}" (confidence: {confidence:.2f}) for human {human_id}')
                                
                                # Check if this gesture should be debounced
                                if self.should_respond_to_gesture(human_id, full_gesture, current_time):
                                    gestures.append(full_gesture)
                                    self.get_logger().info(f'ACCEPTED gesture "{full_gesture}" for human {human_id} [MEDIAPIPE PRIMARY]')
                                else:
                                    self.get_logger().debug(f'REJECTED gesture "{full_gesture}" for human {human_id} (debounced) [MEDIAPIPE]')
            
            return gestures, "mediapipe_primary"
            
        except Exception as e:
            self.get_logger().error(f'Error in MediaPipe gesture recognition: {e}')
            return [], "mediapipe_error"
    
    def is_hand_in_human_bbox_mp(self, hand_landmarks, human_bbox: List[int], frame_shape: Tuple) -> bool:
        """Check if MediaPipe hand landmarks are within human bounding box"""
        try:
            height, width = frame_shape[:2]
            x1, y1, x2, y2 = human_bbox
            
            # Get hand center from landmarks (MediaPipe format)
            hand_x = sum([lm.x for lm in hand_landmarks]) / len(hand_landmarks)
            hand_y = sum([lm.y for lm in hand_landmarks]) / len(hand_landmarks)
            
            # Convert normalized coordinates to pixel coordinates
            hand_x_px = int(hand_x * width)
            hand_y_px = int(hand_y * height)
            
            # Check if hand center is within human bounding box (with some margin)
            margin = 50  # pixels
            return (x1 - margin <= hand_x_px <= x2 + margin and 
                    y1 - margin <= hand_y_px <= y2 + margin)
                    
        except Exception as e:
            self.get_logger().error(f'Error checking MediaPipe hand-human association: {e}')
            return False
    
    def should_respond_to_gesture(self, human_id: int, gesture: str, current_time: float) -> bool:
        """Check if we should respond to this gesture with enhanced stability checking"""
        try:
            # Initialize gesture history for this human if needed
            if human_id not in self.last_gesture_responses:
                self.last_gesture_responses[human_id] = {}
            if human_id not in self.gesture_stability_buffer:
                self.gesture_stability_buffer[human_id] = {}
            
            # Initialize stability buffer for this gesture
            if gesture not in self.gesture_stability_buffer[human_id]:
                self.gesture_stability_buffer[human_id][gesture] = []
            
            # Add current detection to stability buffer
            stability_buffer = self.gesture_stability_buffer[human_id][gesture]
            stability_buffer.append(current_time)
            
            # Clean old entries from stability buffer
            cutoff_time = current_time - self.gesture_stability_window
            stability_buffer[:] = [t for t in stability_buffer if t > cutoff_time]
            
            # Check if gesture is stable enough
            if len(stability_buffer) < self.min_gesture_confidence_frames:
                return False  # Not enough frames to confirm gesture
            
            # Check temporal stability - gesture should be consistent (using measured FPS)
            expected_frames = self.gesture_stability_window * self.measured_fps
            stability_ratio = len(stability_buffer) / max(expected_frames, 1)  # Prevent division by zero
            if stability_ratio < self.gesture_stability_threshold:
                return False  # Gesture not stable enough
            
            # Check if we've responded to this gesture recently (enhanced debouncing)
            if gesture in self.last_gesture_responses[human_id]:
                last_response_time = self.last_gesture_responses[human_id][gesture]
                # Use shorter debounce time for wave gestures (they're dynamic)
                debounce_time = 1.0 if "wave" in gesture else self.gesture_debounce_time
                if current_time - last_response_time < debounce_time:
                    return False  # Too soon, debounce this gesture
            
            # Check if we've responded to ANY gesture from this human recently (global cooldown)
            for prev_gesture, prev_time in self.last_gesture_responses[human_id].items():
                if current_time - prev_time < 0.5:  # 0.5 second global cooldown per human (much reduced)
                    return False
            
            # Record this gesture response
            self.last_gesture_responses[human_id][gesture] = current_time
            
            # Clear stability buffer after successful response
            self.gesture_stability_buffer[human_id][gesture] = []
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error in gesture debouncing: {e}')
            return False  # Default to rejecting gesture on error
    
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
            
            # Convert enum values to strings for JSON serialization
            serializable_states = {}
            for gesture, state in current_states.items():
                serializable_states[gesture] = state.value if hasattr(state, 'value') else str(state)
            
            return serializable_states
            
        except Exception as e:
            self.get_logger().error(f'Error updating gesture states: {e}')
            return {}
    
    
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
                        'gesture_states': gesture_result.get('gesture_states', {}),
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