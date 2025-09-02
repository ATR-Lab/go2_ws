# Human Interaction Package

A ROS2 package providing AI-powered human detection, gesture recognition, and interaction management for the robot dog petting zoo system. This package uses YOLO for human detection and MediaPipe for advanced gesture recognition.

## Purpose

The `human_interaction` package is the **core AI processing component** of the robot dog petting zoo system. It provides:

- **Human Detection**: YOLO-based computer vision to identify people in camera feeds
- **Gesture Recognition**: MediaPipe-powered hand gesture analysis and classification
- **Interaction Management**: State machine for robot behavior coordination
- **Proximity Analysis**: Distance estimation and zone classification
- **Behavioral Intelligence**: Context-aware responses to human presence and gestures

## Package Contents

### Nodes

#### 1. `human_detection_node`
**Purpose**: AI-powered human and gesture detection
- **Executable**: `ros2 run human_interaction human_detection_node`
- **Subscribes**:
  - `/camera/image_raw` (sensor_msgs/Image): Camera feed
  - `/camera/camera_info` (sensor_msgs/CameraInfo): Camera calibration
- **Publishes**:
  - `/human_detection/people` (std_msgs/String): Human detection results (JSON)
  - `/human_detection/gestures` (std_msgs/String): Gesture recognition results (JSON)
  - `/human_detection/proximity` (std_msgs/String): Distance and zone information (JSON)
  - `/human_detection/pose` (std_msgs/String): Body pose data (JSON)

**Parameters**:
- `detection_confidence` (float, default: 0.6): YOLO confidence threshold
- `processing_frequency` (float, default: 10.0): AI processing rate (Hz)
- `max_detection_distance` (float, default: 10.0): Maximum detection range (meters)
- `interaction_zone_distance` (float, default: 2.0): Close interaction zone (meters)
- `approach_zone_distance` (float, default: 5.0): Approach detection zone (meters)
- `model_path` (string): Path to YOLO model file
- `enable_pose_estimation` (bool, default: true): Enable MediaPipe pose detection
- `enable_gesture_recognition` (bool, default: true): Enable hand gesture recognition

#### 2. `interaction_manager_node`
**Purpose**: Robot behavior state machine and interaction coordination
- **Executable**: `ros2 run human_interaction interaction_manager_node`
- **Subscribes**:
  - `/human_detection/people`: Human detection data
  - `/human_detection/gestures`: Gesture recognition data
  - `/human_detection/proximity`: Proximity information
- **Publishes**:
  - `/interaction/state` (std_msgs/String): Current interaction state (JSON)
  - `/interaction/commands` (std_msgs/String): Behavior commands (JSON)
  - `/interaction/events` (std_msgs/String): Interaction events (JSON)
  - `/tts` (std_msgs/String): Text-to-speech requests (JSON)

**Parameters**:
- `state_timeout` (float, default: 30.0): State transition timeout (seconds)
- `max_interaction_time` (float, default: 60.0): Maximum interaction duration (seconds)
- `resume_patrol_delay` (float, default: 5.0): Delay before resuming patrol (seconds)
- `interaction_zone_distance` (float, default: 2.0): Close interaction threshold (meters)
- `approach_zone_distance` (float, default: 5.0): Approach detection threshold (meters)
- `enable_automatic_resume` (bool, default: true): Auto-resume patrol mode
- `enable_gesture_responses` (bool, default: true): Respond to gestures
- `enable_speech_responses` (bool, default: true): Enable TTS responses

#### 3. `gesture_recognition_node`
**Purpose**: Advanced gesture analysis and sequence detection
- **Executable**: `ros2 run human_interaction gesture_recognition_node`
- **Subscribes**:
  - `/human_detection/pose`: Body pose data
  - `/human_detection/gestures`: Basic gesture data
- **Publishes**:
  - `/gesture_recognition/detailed_gestures`: Advanced gesture analysis
  - `/gesture_recognition/gesture_sequences`: Gesture pattern detection
  - `/gesture_recognition/gesture_confidence`: Confidence scoring

## AI Capabilities

### Human Detection (YOLO)
**Model**: YOLOv8 Nano (`yolov8n.pt`)
- **Detection Class**: Humans (class 0)
- **Confidence Scoring**: 0.0-1.0 range
- **Bounding Box**: Pixel coordinates (x1, y1, x2, y2)
- **Distance Estimation**: Approximate distance based on bounding box size
- **Zone Classification**: 
  - `interaction` (< 2m): Close engagement zone
  - `approach` (2-5m): Approaching person
  - `far` (> 5m): Distant detection

### Gesture Recognition (MediaPipe)
**Engine**: MediaPipe Hands + Custom Analysis
- **Hand Detection**: Up to 2 hands simultaneously
- **Landmark Extraction**: 21 hand landmarks per hand
- **Gesture Classification**: Real-time gesture analysis

#### Supported Gestures
| Gesture | Description | Detection Logic |
|---------|-------------|-----------------|
| 👍 `thumbs_up` | Thumb extended, other fingers closed | Only thumb up |
| 👉 `pointing` | Index finger extended | Only index finger up |
| ✌️ `peace_sign` | Index and middle fingers extended | Index + middle up |
| 🤘 `rock_sign` | Index and pinky extended | Index + pinky up |
| ✋ `open_hand` | All fingers extended | 4+ fingers up |
| ✊ `fist` | All fingers closed | No fingers up |
| 👌 `ok_sign` | Thumb-index circle, others up | Thumb-index close + others up |
| 👋 `wave` | Hand raised above wrist | Hand elevated + fingers up |
| 👀 `hands_visible` | Hands detected, no specific gesture | Fallback detection |

#### Gesture Sequences
- **Repeated Gestures**: Same gesture 3+ times in 1 second
- **Gesture Combinations**: 
  - `wave_then_point`: Wave followed by pointing
  - `thumbs_up_then_peace`: Thumbs up followed by peace sign
- **Temporal Analysis**: 2-second gesture history tracking

### Interaction States
The system manages these behavioral states:

| State | Description | Triggers | Actions |
|-------|-------------|----------|---------|
| `PATROL` | Default roaming behavior | No humans detected | Continue navigation |
| `HUMAN_DETECTED` | Person spotted | Human enters approach zone | Stop, assess situation |
| `INTERACTION` | Active engagement | Human in interaction zone | Gesture recognition, responses |
| `COMMAND_EXECUTION` | Processing gesture command | Specific gesture detected | Execute corresponding behavior |
| `RESUME_PATROL` | Returning to patrol | Interaction timeout/completion | Resume navigation |

## Data Formats

### Human Detection Output
```json
{
  "timestamp": 1234567890.123,
  "detection": {
    "bbox": [x1, y1, x2, y2],
    "confidence": 0.85,
    "class_id": 0,
    "class_name": "person"
  },
  "distance": 3.2,
  "zone": "approach",
  "frame_id": "camera_frame"
}
```

### Gesture Recognition Output
```json
{
  "timestamp": 1234567890.123,
  "gestures": ["left_thumbs_up", "right_wave"],
  "sequences": ["repeated_left_thumbs_up"],
  "hand_count": 2,
  "confidence": {
    "left_thumbs_up": 0.92,
    "right_wave": 0.87
  }
}
```

### Interaction State Output
```json
{
  "timestamp": 1234567890.123,
  "current_state": "interaction",
  "previous_state": "human_detected",
  "state_duration": 5.2,
  "detected_humans": 1,
  "active_gestures": ["thumbs_up"],
  "next_action": "respond_to_gesture"
}
```

## Quick Start

### 1. Basic Human Detection
```bash
# Ensure camera feed is available
ros2 topic echo /camera/image_raw

# Launch human detection
ros2 run human_interaction human_detection_node

# Monitor detection results
ros2 topic echo /human_detection/people
```

### 2. Complete Interaction System
```bash
# Launch all interaction nodes
ros2 run human_interaction human_detection_node &
ros2 run human_interaction interaction_manager_node &
ros2 run human_interaction gesture_recognition_node &

# Monitor interaction state
ros2 topic echo /interaction/state
```

### 3. Gesture Testing
```bash
# Launch detection with gesture focus
ros2 run human_interaction human_detection_node --ros-args -p enable_gesture_recognition:=true

# Monitor gestures
ros2 topic echo /human_detection/gestures
```

## Configuration

### Model Setup
The system requires YOLO model files:
```bash
# Default model location
/home/atr-lab/ros2_ws/models/yolov8n.pt

# Alternative: Package share directory
<package_share>/models/yolov8n.pt
```

### Parameter Tuning
```bash
# Adjust detection sensitivity
ros2 run human_interaction human_detection_node --ros-args -p detection_confidence:=0.7

# Change processing frequency
ros2 run human_interaction human_detection_node --ros-args -p processing_frequency:=15.0

# Modify interaction zones
ros2 run human_interaction interaction_manager_node --ros-args -p interaction_zone_distance:=1.5
```

## Performance Optimization

### Hardware Requirements
- **GPU**: NVIDIA GPU recommended for YOLO inference
- **CPU**: Multi-core processor for MediaPipe processing
- **Memory**: 4GB+ RAM for model loading
- **Camera**: 30 FPS capable webcam or robot camera

### Optimization Tips
```bash
# Reduce processing frequency for lower-end hardware
-p processing_frequency:=5.0

# Lower detection confidence for more detections
-p detection_confidence:=0.5

# Disable pose estimation if not needed
-p enable_pose_estimation:=false
```

### Performance Monitoring
```bash
# Check processing rates
ros2 topic hz /human_detection/people

# Monitor system resources
htop
nvidia-smi  # For GPU usage
```

## Integration

### With Test Camera Package
```bash
# Terminal 1: Camera
ros2 launch test_camera simple_camera_test.launch.py

# Terminal 2: AI Processing
ros2 run human_interaction human_detection_node
ros2 run human_interaction interaction_manager_node
```

### With Go2 Robot SDK
```bash
# Robot provides camera feed to /camera/image_raw
# Human interaction processes and publishes behavior commands
# Robot SDK subscribes to /interaction/commands for behavior execution
```

### With Speech Processing
```bash
# Interaction manager publishes to /tts
# Speech processor converts to audio commands
# Audio sent to robot via WebRTC
```

## Troubleshooting

### Model Loading Issues
```bash
# Check model file exists
ls -la /home/atr-lab/ros2_ws/models/yolov8n.pt

# Download model if missing
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt -O models/yolov8n.pt
```

### No Human Detection
- Check camera feed: `ros2 topic echo /camera/image_raw`
- Verify lighting conditions
- Adjust detection confidence: `-p detection_confidence:=0.4`
- Check YOLO model compatibility

### Gesture Recognition Issues
- Ensure hands are clearly visible
- Check MediaPipe installation: `python3 -c "import mediapipe; print('OK')"`
- Verify hand landmarks: Enable debug logging
- Test with simple gestures first (thumbs up, open hand)

### Performance Issues
```bash
# Check CPU usage
top -p $(pgrep -f human_detection_node)

# Reduce processing load
-p processing_frequency:=5.0
-p enable_pose_estimation:=false
```

### State Machine Issues
```bash
# Monitor state transitions
ros2 topic echo /interaction/state

# Check parameter configuration
ros2 param list /interaction_manager_node
ros2 param get /interaction_manager_node state_timeout
```

## Development

### Adding New Gestures
1. Modify `analyze_hand_gesture()` in `human_detection_node.py`
2. Add gesture logic based on hand landmarks
3. Update gesture classification in `recognize_gestures()`
4. Test with various hand positions and lighting

### Extending Interaction States
1. Add new state to `InteractionState` enum
2. Implement state transition logic in `interaction_manager_node.py`
3. Add corresponding behavior commands
4. Update state machine documentation

### Custom Behavior Responses
1. Subscribe to `/human_detection/gestures` in your node
2. Implement gesture-specific behavior logic
3. Publish commands to robot control topics
4. Coordinate with interaction manager state

## Dependencies

### ROS2 Packages
- `rclpy`: Python ROS2 client library
- `sensor_msgs`: Image and camera messages
- `std_msgs`: String messages for JSON data
- `geometry_msgs`: Pose and point messages
- `cv_bridge`: OpenCV-ROS2 integration

### Python Libraries
- `ultralytics`: YOLOv8 implementation
- `mediapipe`: Google's ML framework for pose/hand detection
- `opencv-python`: Computer vision library
- `numpy`: Numerical computing
- `json`: Data serialization
- `threading`: Concurrent processing

### AI Models
- **YOLOv8 Nano**: Human detection model (~6MB)
- **MediaPipe Hands**: Hand landmark detection
- **MediaPipe Pose**: Body pose estimation

## See Also
- [`test_camera` package](../test_camera/README.md): Camera testing and visualization
- [Robot Dog Petting Zoo Documentation](../../PHASE1_IMPLEMENTATION_README.md): Complete system overview
- [Go2 Robot SDK Integration](../go2_robot_sdk/README.md): Robot hardware interface