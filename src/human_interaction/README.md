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
  - `/human_detection/people` (std_msgs/String): Human detection results with tracking IDs (JSON)
  - `/human_detection/gestures` (std_msgs/String): Per-human gesture recognition results (JSON)
  - `/human_detection/combined_gestures` (std_msgs/String): Priority-ordered multi-human gestures (JSON)
  - `/human_detection/proximity` (std_msgs/String): Distance and zone information (JSON)
  - `/human_detection/pose` (geometry_msgs/PoseArray): Body pose landmarks

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
  - `/human_detection/people`: Human detection data with tracking IDs
  - `/human_detection/combined_gestures`: Priority-ordered multi-human gestures
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



## AI Capabilities

### Human Detection & Tracking
**Detection Model**: YOLOv8 Nano (`yolov8n.pt`)
**Tracking Algorithm**: ByteTrack (robust multi-object tracking)

- **Detection Class**: Humans (class 0)
- **Confidence Scoring**: 0.0-1.0 range
- **Bounding Box**: Pixel coordinates (x1, y1, x2, y2)
- **Persistent Human IDs**: Maintains consistent tracking across frames
- **Laggy Video Handling**: Optimized for WebRTC feeds with variable timing
- **Dynamic FPS Adaptation**: Automatically adjusts to measured frame rates
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

#### Gesture Sequences & States
- **Gesture States**: NEW → ONGOING → ENDED lifecycle tracking
- **Temporal Smoothing**: Adaptive stability windows for laggy video
- **Multi-Human Attribution**: Gestures correctly assigned to specific humans
- **Debouncing**: Prevents gesture spam with per-human cooldowns
- **Priority System**: Closer humans get higher gesture priority
- **Repeated Gestures**: Same gesture 3+ times in 1 second
- **Gesture Combinations**: 
  - `wave_then_point`: Wave followed by pointing
  - `thumbs_up_then_peace`: Thumbs up followed by peace sign
- **Temporal Analysis**: 2-second gesture history tracking

### Smart Interaction State Machine
The system manages these behavioral states with intelligent multi-human handling:

| State | Description | Triggers | Actions |
|-------|-------------|----------|---------|
| `PATROL` | Default roaming behavior | No humans detected | Continue navigation |
| `HUMAN_DETECTED` | Person spotted | Human enters approach zone | Stop, assess situation |
| `INTERACTION` | Active engagement | Human in interaction zone | Gesture recognition, responses |
| `COMMAND_EXECUTION` | Processing gesture command | Specific gesture detected | Execute corresponding behavior |
| `RESUME_PATROL` | Returning to patrol | Interaction timeout/completion | Resume navigation |

#### Smart Features
- **Multi-Human Management**: Handles multiple people simultaneously with priority queues
- **Context-Aware Greetings**: Recognizes returning humans and adjusts responses
- **Human Memory**: Tracks interaction history per person (first_seen, last_interaction, total_interactions)
- **Return Detection**: Differentiates between quick returns, short returns, and first-time visitors
- **Command Rate Limiting**: Prevents spam with per-human, per-gesture cooldowns
- **State Transition Smoothing**: Intelligent handoffs between multiple human interactions
- **Proximity-Based Priority**: Closer humans get higher interaction priority

## Data Formats

### Human Detection Output (with Tracking)
```json
{
  "timestamp": 1234567890.123,
  "detection": {
    "id": 42,
    "bbox": [x1, y1, x2, y2],
    "confidence": 0.85,
    "class": "human",
    "center": [cx, cy],
    "area": 15000
  },
  "distance": 3.2,
  "zone": "approach"
}
```

### Gesture Recognition Output (Per-Human)
```json
{
  "timestamp": 1234567890.123,
  "human_id": 42,
  "gestures": ["left_thumbs_up", "right_wave"],
  "gesture_states": {
    "left_thumbs_up": "new",
    "right_wave": "ongoing"
  }
}
```

### Combined Multi-Human Gestures
```json
{
  "timestamp": 1234567890.123,
  "prioritized_gestures": [
    {
      "human_id": 42,
      "gestures": ["left_thumbs_up"],
      "gesture_states": {"left_thumbs_up": "new"},
      "priority_score": 15000.85
    },
    {
      "human_id": 37,
      "gestures": ["right_wave"],
      "gesture_states": {"right_wave": "ongoing"},
      "priority_score": 8500.72
    }
  ],
  "total_humans": 2
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
# Launch core interaction nodes
ros2 run human_interaction human_detection_node &
ros2 run human_interaction interaction_manager_node &

# Monitor interaction state
ros2 topic echo /interaction/state

# Monitor multi-human gestures
ros2 topic echo /human_detection/combined_gestures
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
- **Camera**: Variable FPS camera (system adapts to 3-30 FPS)
- **Network**: Stable connection for WebRTC robot feeds

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
- Monitor FPS: `ros2 topic hz /camera/image_raw`
- Check ByteTrack tracking: Look for consistent human IDs

### Gesture Recognition Issues
- Ensure hands are clearly visible
- Check MediaPipe installation: `python3 -c "import mediapipe; print('OK')"`
- Verify hand landmarks: Enable debug logging
- Test with simple gestures first (thumbs up, open hand)
- Check gesture attribution: Ensure hands are within human bounding boxes
- Monitor gesture states: Look for NEW → ONGOING → ENDED transitions
- Verify FPS adaptation: System should adapt to measured frame rates

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
5. Consider gesture state transitions (NEW/ONGOING/ENDED)
6. Test multi-human scenarios for proper attribution

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
- `boxmot`: ByteTrack and other SOTA tracking algorithms

### AI Models & Algorithms
- **YOLOv8 Nano**: Human detection model (~6MB)
- **ByteTrack**: Multi-object tracking for persistent human IDs
- **MediaPipe Hands**: Hand landmark detection with gesture analysis
- **MediaPipe Pose**: Body pose estimation (optional)
- **Custom State Machine**: Smart multi-human interaction management

## See Also
- [`test_camera` package](../test_camera/README.md): Camera testing and visualization
- [Robot Dog Petting Zoo Documentation](../../PHASE1_IMPLEMENTATION_README.md): Complete system overview
- [Go2 Robot SDK Integration](../go2_robot_sdk/README.md): Robot hardware interface