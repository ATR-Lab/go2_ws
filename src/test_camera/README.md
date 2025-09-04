# Test Camera Package

A ROS2 package providing camera capture and visualization utilities for testing the robot dog petting zoo system. This package enables development and testing without requiring the physical Go2 robot by using a PC's USB camera.

## Purpose

The `test_camera` package serves as a **development and testing tool** for the robot dog petting zoo system. It provides:

- **USB Camera Interface**: Captures video from PC webcam and publishes to ROS2 topics
- **Visual Display**: Shows camera feed with AI detection overlays (YOLO + MediaPipe)
- **Development Environment**: Allows testing human detection and gesture recognition offline
- **Integration Testing**: Validates the complete AI pipeline before deploying to the robot

## Package Contents

### Nodes

#### 1. `test_camera_node`
**Purpose**: USB camera capture and ROS2 publishing with optimized coordinate system
- **Executable**: `ros2 run test_camera test_camera_node`
- **Publishes**: 
  - `/camera/image_raw` (sensor_msgs/Image): Raw camera frames (no flipping for AI processing)
  - `/camera/camera_info` (sensor_msgs/CameraInfo): Camera calibration data
- **Parameters**:
  - `camera_index` (int, default: 0): USB camera device index
  - `frame_rate` (float, default: 30.0): Capture frame rate (Hz)
  - `image_width` (int, default: 640): Frame width (pixels)
  - `image_height` (int, default: 480): Frame height (pixels)
  - `enable_preview` (bool, default: true): Show OpenCV preview window
- **Coordinate System**: Publishes original camera orientation for accurate AI processing

#### 2. `simple_camera_display`
**Purpose**: Enhanced multi-human camera display with AI overlays and optimized coordinate system
- **Executable**: `ros2 run test_camera simple_camera_display`
- **Subscribes**:
  - `/camera/image_raw`: Camera feed
  - `/human_detection/people`: YOLO human detection results (per-human)
  - `/human_detection/gestures`: MediaPipe gesture recognition (per-human)
  - `/human_detection/combined_gestures`: Priority-ordered multi-human gestures
  - `/interaction/state`: Robot interaction state
  - `/interaction/events`: Interaction events with human IDs
- **Enhanced Features**:
  - **Multi-Human Support**: Displays multiple humans with unique IDs
  - **Gesture Attribution**: Shows which human made which gesture with MediaPipe accuracy
  - **Gesture State Tracking**: Visual indication of NEW/ONGOING/ENDED gestures
  - **Priority Visualization**: Shows gesture priority queue and scores
  - **Interaction Events**: Recent interaction history with human IDs
  - **Per-Human Overlays**: Individual bounding boxes with gesture states
  - **Real-time Updates**: Live tracking of human interactions
  - **Performance Metrics**: FPS, detection counts, human tracking
  - **Screenshot Capability**: Save current frame (press 's')
  - **Optimized Display**: Coordinate-preserving flip for accurate overlays with natural text
- **Visual Elements**:
  - **Green Bounding Boxes**: Human detection with ID labels (coordinate-aligned)
  - **Color-Coded Gestures**: NEW (Cyan), ONGOING (Orange), ENDED (Gray)
  - **Priority Queue**: Top 5 prioritized gestures with scores
  - **Event History**: Last 3 interaction events
  - **Waiting Screen**: Friendly UI when no camera feed available
  - **Mirror Display**: Natural mirror-like view for user interaction
- **Technical Improvements**:
  - **Coordinate System Preservation**: Bounding boxes drawn before image flip for accuracy
  - **Text Rendering**: Text overlays drawn after flip for normal appearance
  - **Performance Optimization**: Efficient single-flip operation for natural display
- **Controls**:
  - `q`: Quit application
  - `s`: Save screenshot



### Launch Files

#### 1. `simple_camera_test.launch.py`
**Purpose**: Basic camera testing
```bash
ros2 launch test_camera simple_camera_test.launch.py
```
**Launches**:
- `test_camera_node` (camera capture)
- `simple_camera_display` (enhanced display)

**Use Case**: Quick camera and display testing

#### 2. `test_system_rviz.launch.py`
**Purpose**: Complete AI system testing
```bash
ros2 launch test_camera test_system_rviz.launch.py
```
**Parameters**:
- `enable_camera` (default: true): Launch camera node
- `enable_detection` (default: true): Launch human detection
- `enable_interaction` (default: true): Launch interaction manager
- `enable_visualizer` (default: true): Launch RViz visualizer
- `enable_rviz` (default: true): Launch RViz2 application

**Use Case**: Full system integration testing

## Quick Start

### 1. Basic Camera Test
```bash
# Terminal 1: Launch camera and display
ros2 launch test_camera simple_camera_test.launch.py
```

### 2. Complete AI System Test
```bash
# Terminal 1: Camera + Display
ros2 launch test_camera simple_camera_test.launch.py

# Terminal 2: AI Processing (human detection + interaction)
ros2 launch test_camera test_system_rviz.launch.py enable_camera:=false enable_visualizer:=false enable_rviz:=false
```

### 3. Individual Node Testing
```bash
# Camera only
ros2 run test_camera test_camera_node

# Display only (requires camera feed from another source)
ros2 run test_camera simple_camera_display
```

## Expected Behavior

### Multi-Human Camera Display Features
- **Live Video**: Real-time camera feed from USB webcam
- **Multi-Human Detection**: Green bounding boxes with unique IDs (Human 1, Human 2, etc.)
- **Per-Human Gesture Recognition**: Individual gesture tracking with state information:
  - **Left/Right Hand Gestures**: `left_wave`, `right_thumbs_up`, `left_pointing`, etc.
  - **Gesture States**: NEW (Cyan), ONGOING (Orange), ENDED (Gray)
  - **Attribution**: Each gesture clearly linked to specific human ID
- **Priority Visualization**: Top-left shows multi-human gesture queue:
  - Priority ranking (#1, #2, #3...)
  - Priority scores (based on proximity and confidence)
  - Active interaction human highlighted
- **Interaction Events**: Recent interaction history:
  - `interaction_started (Human 2)`
  - `interaction_ended (Human 1)`
  - Event timestamps and human IDs
- **Enhanced Interaction State**: Top-right corner shows:
  - Current robot state (`patrol`, `human_detected`, `interaction`)
  - Active interaction human ID
- **Performance Metrics**: Bottom-left displays:
  - FPS counter
  - Total humans detected
  - Active gestures count
  - Tracking performance

### Gesture Recognition
The system uses **MediaPipe Gesture Recognizer** for accurate hand gesture detection:

**MediaPipe Built-in Gestures**:
- **👍 Thumbs Up**: Thumb extended upward (MediaPipe: `thumb_up`)
- **👎 Thumbs Down**: Thumb extended downward (MediaPipe: `thumb_down`)
- **👉 Pointing**: Index finger pointing (MediaPipe: `pointing_up`)
- **✌️ Peace Sign**: Victory gesture (MediaPipe: `victory`)
- **✊ Fist**: Closed fist (MediaPipe: `closed_fist`)
- **✋ Open Hand**: Open palm (MediaPipe: `open_palm`)
- **🤘 Rock Sign**: I Love You sign (MediaPipe: `iloveyou`)
- **👀 Hands Visible**: Hands detected without specific gesture

**Detection Features**:
- **Automatic Hand Labeling**: Correctly identifies left/right hands
- **Camera Compatibility**: Works with both mirrored and non-mirrored feeds
- **Multi-Human Attribution**: Gestures assigned to correct humans
- **Confidence Filtering**: 0.6 threshold for reliable detection

## Troubleshooting

### Camera Issues
```bash
# Check available cameras
ls /dev/video*

# Test different camera index
ros2 run test_camera test_camera_node --ros-args -p camera_index:=1
```

### No Gesture Recognition
- Ensure good lighting conditions
- Keep hands clearly visible in camera view
- Make distinct, deliberate gestures
- Check that `human_interaction` package is running

### Display Issues
- The `simple_camera_display` uses OpenCV for reliable cross-platform compatibility
- Ensure X11 forwarding is enabled if using SSH

### Performance Issues
```bash
# Reduce frame rate
ros2 run test_camera test_camera_node --ros-args -p frame_rate:=15.0

# Reduce resolution
ros2 run test_camera test_camera_node --ros-args -p image_width:=320 -p image_height:=240
```

## Dependencies

### ROS2 Packages
- `rclpy`: Python ROS2 client library
- `sensor_msgs`: Image and camera info messages
- `cv_bridge`: OpenCV-ROS2 bridge
- `python_qt_binding`: Qt integration (for advanced visualizer)

### Python Libraries
- `opencv-python`: Computer vision (provided by ROS2)
- `numpy`: Numerical computing
- `json`: Data serialization

### System Requirements
- USB webcam or built-in camera
- OpenCV-compatible camera drivers
- Sufficient lighting for gesture recognition

## Integration

This package integrates with:
- **`human_interaction`**: Provides AI processing (YOLO + MediaPipe)
- **`go2_robot_sdk`**: Can replace test camera with robot camera feed
- **RViz2**: For advanced visualization and debugging

## Development Notes

### Camera Topic Compatibility
The package publishes to `/camera/image_raw` which is compatible with:
- Go2 robot camera feed
- Standard ROS2 camera drivers
- Other vision processing nodes

### Multi-Human Testing Scenarios

#### Scenario 1: Single Human Interaction
```bash
# Test basic human detection and gesture recognition
ros2 launch test_camera simple_camera_test.launch.py
ros2 run human_interaction human_detection_node
ros2 run human_interaction interaction_manager_node
```
**Expected**: Single human gets ID 1, gestures show NEW→ONGOING→ENDED states

#### Scenario 2: Multiple Humans - Priority Testing
```bash
# Same setup as above, but have 2-3 people in camera view
```
**Expected**: 
- Each person gets unique ID (Human 1, 2, 3...)
- Priority queue shows closest person first
- Only highest priority human gets robot responses
- Other humans wait in queue

#### Scenario 3: Continuous Gesture Testing
```bash
# Have someone wave continuously for 30+ seconds
```
**Expected**:
- First wave shows as NEW (Cyan) → robot responds
- Continued waving shows as ONGOING (Orange) → robot ignores
- Stop waving → gesture shows as ENDED (Gray)
- Start waving again → NEW gesture → robot responds again

#### Scenario 4: Gesture Attribution Testing
```bash
# Have multiple people gesture simultaneously
```
**Expected**:
- Each gesture clearly attributed to correct human ID
- Bounding boxes show per-human gesture states
- Priority queue shows all gestures with scores
- No gesture confusion between humans

### Testing Workflow
1. **Development**: Use `simple_camera_test.launch.py` for basic testing
2. **Single Human AI**: Test gesture recognition and state tracking
3. **Multi-Human AI**: Test priority system and gesture attribution  
4. **Integration**: Test with full robot system and command execution
4. **Deployment**: Replace with actual robot camera

### Extending Functionality
- Add new gesture types in `human_interaction` package
- Modify display overlays in `simple_camera_display.py`
- Create custom launch configurations for specific test scenarios

## See Also
- [`human_interaction` package](../human_interaction/README.md): AI processing and gesture recognition
- [Robot Dog Petting Zoo Documentation](../../PHASE1_IMPLEMENTATION_README.md): Complete system overview