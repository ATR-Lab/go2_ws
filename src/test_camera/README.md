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

#### 2. `test_camera_threaded_node`
**Purpose**: High-performance multi-threaded camera capture with optimized QoS for smooth video display
- **Executable**: `ros2 run test_camera test_camera_threaded_node`
- **Architecture**: Multi-threaded design with dedicated capture and publishing threads
- **Publishes**: 
  - `/camera/image_raw` (sensor_msgs/Image): Raw camera frames with RELIABLE QoS
  - `/camera/camera_info` (sensor_msgs/CameraInfo): Camera calibration data
- **Parameters**:
  - `camera_index` (int, default: 0): USB camera device index
  - `frame_rate` (float, default: 30.0): Target publishing rate (Hz)
  - `image_width` (int, default: 640): Frame width (pixels)
  - `image_height` (int, default: 480): Frame height (pixels)
  - `enable_preview` (bool, default: false): Show OpenCV preview window
- **Performance Features**:
  - **Multi-Backend Support**: V4L2, GStreamer, and auto-detect fallback
  - **Camera Warm-up**: 5-frame initialization for stable capture
  - **Frame Staleness Detection**: Skips frames older than 100ms
  - **Optimized Memory**: Direct frame assignment without copying
  - **RELIABLE QoS**: Guaranteed frame delivery for smooth video display

#### 3. `simple_camera_display`
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
**Purpose**: Basic camera testing with original single-threaded architecture
```bash
ros2 launch test_camera simple_camera_test.launch.py
```
**Launches**:
- `test_camera_node` (single-threaded camera capture)
- `simple_camera_display` (enhanced display)

**Use Case**: Legacy testing and development baseline

#### 2. `threaded_camera_only.launch.py`
**Purpose**: High-performance threaded camera node (headless)
```bash
ros2 launch test_camera threaded_camera_only.launch.py
# Or with specific camera:
ros2 launch test_camera threaded_camera_only.launch.py camera_index:=4
```
**Parameters**:
- `camera_index` (default: 4): Camera device index (0=laptop, 4=Logitech BRIO)

**Launches**:
- `test_camera_threaded_node` (multi-threaded camera with RELIABLE QoS)

**Use Case**: Production camera feed for AI processing and RViz display

## Quick Start

### 1. High-Performance Camera (Recommended)
```bash
# Terminal 1: Launch optimized threaded camera node
ros2 launch test_camera threaded_camera_only.launch.py

# Terminal 2: Launch Qt-based UI for smooth video display
ros2 launch test_camera_ui camera_ui.launch.py

# Or launch both together:
ros2 launch test_camera_ui full_camera_system.launch.py
```

### 2. Camera Selection
```bash
# Use laptop camera (index 0)
ros2 launch test_camera threaded_camera_only.launch.py camera_index:=0

# Use external USB camera (index 4 - Logitech BRIO)
ros2 launch test_camera threaded_camera_only.launch.py camera_index:=4

# Check available cameras first
ros2 run test_camera test_camera_threaded_node --ros-args -p camera_index:=0
# (Check logs for "Found camera at index X" messages)
```

### 3. RViz Visualization
```bash
# Terminal 1: Launch threaded camera
ros2 launch test_camera threaded_camera_only.launch.py

# Terminal 2: Launch RViz and add Image display
rviz2
# In RViz: Add -> By display type -> Image
# Set topic to: /camera/image_raw
```

### 4. Legacy Testing (Single-threaded)
```bash
# Basic camera test with original architecture
ros2 launch test_camera simple_camera_test.launch.py
```

### 5. Individual Node Testing
```bash
# Threaded camera only (headless)
ros2 run test_camera test_camera_threaded_node

# Qt UI only (requires camera feed from another source)
ros2 run test_camera_ui test_camera_ui_node

# Legacy camera only
ros2 run test_camera test_camera_node

# Legacy display only
ros2 run test_camera simple_camera_display
```

### 6. AI System Integration
```bash
# Terminal 1: High-performance camera
ros2 launch test_camera threaded_camera_only.launch.py

# Terminal 2: Human detection and gesture recognition
ros2 run human_interaction human_detection_node

# Terminal 3: Interaction manager
ros2 run human_interaction interaction_manager_node

# Terminal 4: Qt UI for monitoring
ros2 launch test_camera_ui camera_ui.launch.py
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

# Test different camera index with threaded node
ros2 run test_camera test_camera_threaded_node --ros-args -p camera_index:=1

# Check camera discovery logs
ros2 run test_camera test_camera_threaded_node --ros-args -p camera_index:=0
# Look for "Found camera at index X" messages
```

### Performance Issues

#### Choppy Video in RViz/UI
```bash
# Ensure using RELIABLE QoS (should be default in threaded node)
# Check QoS settings in test_camera_threaded_node.py

# Verify threaded architecture is being used
ros2 run test_camera test_camera_threaded_node
# Look for "Camera capture and publishing threads started" message
```

#### Slow Camera Capture Warnings
```bash
# Check if camera is being used by another application
sudo lsof /dev/video*

# Try different camera backends (logged automatically)
# V4L2 -> GStreamer -> Auto-detect fallback

# Monitor capture performance
ros2 run test_camera test_camera_threaded_node --ros-args --log-level debug
```

#### Qt UI Issues
```bash
# If Qt platform plugin errors occur:
# Ensure opencv-python-headless is installed (not opencv-python)
pip list | grep opencv

# Check virtual environment setup
source activate_workspace.sh
# Should show "PyQt5 correctly absent from venv"
```

### Camera Selection Issues
```bash
# Laptop camera not working
ros2 launch test_camera threaded_camera_only.launch.py camera_index:=0

# External USB camera not detected
# Check USB connection and power
lsusb | grep -i camera
# Try different USB ports

# Logitech BRIO specific
ros2 launch test_camera threaded_camera_only.launch.py camera_index:=4
```

### No Gesture Recognition
- Ensure good lighting conditions
- Keep hands clearly visible in camera view
- Make distinct, deliberate gestures
- Check that `human_interaction` package is running
- Verify camera feed is reaching AI nodes:
  ```bash
  ros2 topic echo /camera/image_raw --once
  ```

### Display Issues
- **Qt UI**: Uses `test_camera_ui` package for professional display
- **Legacy OpenCV**: Uses `simple_camera_display` (may not work with headless OpenCV)
- Ensure X11 forwarding is enabled if using SSH

### Architecture Migration Issues
```bash
# If legacy nodes fail after switching to headless OpenCV:
# This is expected - use threaded architecture instead

# Legacy single-threaded (may have blocking issues):
ros2 launch test_camera simple_camera_test.launch.py

# Modern threaded (recommended):
ros2 launch test_camera threaded_camera_only.launch.py
ros2 launch test_camera_ui camera_ui.launch.py
```

### Performance Tuning
```bash
# Reduce frame rate if needed
ros2 run test_camera test_camera_threaded_node --ros-args -p frame_rate:=15.0

# Reduce resolution for lower bandwidth
ros2 run test_camera test_camera_threaded_node --ros-args -p image_width:=320 -p image_height:=240

# Disable preview for headless operation
ros2 run test_camera test_camera_threaded_node --ros-args -p enable_preview:=false
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

## Performance Optimization & Architecture Evolution

### The Choppiness Investigation Journey

This package underwent significant performance optimization to solve video choppiness issues. Here's what we learned:

#### Problem: Choppy Video Display
- **Symptom**: Video appeared choppy in RViz and display nodes, despite good camera hardware
- **Initial Hypothesis**: Camera hardware blocking or CPU overload
- **Reality**: ROS implementation and QoS configuration issues

#### Root Cause Analysis

**Hardware Performance Testing**:
```bash
# System Python test: 29.5 FPS, 0.1% slow frames - PERFECT
python3 -c "import cv2; cap = cv2.VideoCapture(4); [cap.read() for _ in range(1000)]"

# Virtual Environment test: 29.0 FPS, 0.1% slow frames - PERFECT  
python -c "import cv2; cap = cv2.VideoCapture(4); [cap.read() for _ in range(1000)]"

# ROS Node: Choppy, intermittent 100-130ms captures - PROBLEMATIC
```

**Key Finding**: Hardware was perfect; the issue was ROS-specific implementation problems.

#### Solution: Multi-Threaded Architecture + RELIABLE QoS

**Architecture Changes**:
1. **Dedicated Capture Thread**: Isolates camera hardware access from ROS executor
2. **Dedicated Publishing Thread**: Controls ROS message publishing rate independently
3. **Thread-Safe Frame Buffer**: Lock-protected shared memory between threads
4. **Multiple Backend Support**: V4L2 → GStreamer → Auto-detect fallback
5. **Camera Warm-up**: 5-frame initialization for stable capture
6. **Memory Optimization**: Direct frame assignment without copying

**Critical QoS Discovery**:
- **BEST_EFFORT QoS**: Caused frame drops under load → choppy video
- **RELIABLE QoS**: Guaranteed frame delivery → smooth video display

### QoS Policy Guide: When to Use RELIABLE vs BEST_EFFORT

#### RELIABLE QoS - Use For:
✅ **Video Display Applications**
- **Human viewing**: Smooth playback requires every frame
- **RViz visualization**: Complete frame sequences for analysis
- **Recording/streaming**: No missing frames acceptable
- **UI applications**: Consistent user experience

✅ **Sequence-Critical Computer Vision**
- **Human tracking**: Missing frames break tracking continuity
- **Gesture recognition**: Complete gesture sequences needed
- **Motion analysis**: Temporal continuity required
- **Security applications**: Cannot miss critical events

**Configuration**:
```python
camera_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10  # Buffer for guaranteed delivery
)
```

#### BEST_EFFORT QoS - Use For:
✅ **Real-Time Computer Vision**
- **Object detection**: Latest frame is sufficient
- **Real-time response**: Low latency more important than completeness
- **High-frequency sensors**: Occasional drops acceptable
- **Resource-constrained systems**: Lower overhead needed

✅ **Sensor Data Streams**
- **Lidar scans**: Next scan replaces previous data
- **IMU readings**: High frequency, occasional loss OK
- **Robot pose**: Latest position most relevant

**Configuration**:
```python
camera_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1  # Only keep latest frame
)
```

### Performance Comparison

| Architecture | FPS | Frame Drops | CPU Usage | Use Case |
|-------------|-----|-------------|-----------|----------|
| **Single-threaded + BEST_EFFORT** | 8-12 | High | Variable | ❌ Choppy display |
| **Single-threaded + RELIABLE** | 15-20 | Medium | High | ⚠️ Better but blocking |
| **Multi-threaded + BEST_EFFORT** | 25-28 | Low | Low | ✅ Real-time AI |
| **Multi-threaded + RELIABLE** | 28-30 | None | Low | ✅ Smooth display |

### Architecture Recommendations

#### For Video Display (Human Viewing):
```bash
# Use threaded node with RELIABLE QoS
ros2 launch test_camera threaded_camera_only.launch.py
ros2 run test_camera_ui test_camera_ui_node
```

#### For Computer Vision Processing:
```bash
# Option 1: Use existing human_interaction (BEST_EFFORT)
ros2 run human_interaction human_detection_node

# Option 2: Modify to RELIABLE for sequence analysis
# Edit human_detection_node.py QoS settings
```

#### For Development/Testing:
```bash
# Legacy single-threaded for baseline comparison
ros2 launch test_camera simple_camera_test.launch.py
```

### Key Learnings

1. **QoS Choice is Use-Case Dependent**: Same data type needs different QoS for different applications
2. **Threading Solves Blocking**: Dedicated threads prevent camera hardware from blocking ROS executor
3. **Hardware Testing is Critical**: Always verify hardware performance before optimizing software
4. **RELIABLE ≠ Slower**: RELIABLE QoS can actually provide smoother performance for display applications
5. **Buffer Depth Matters**: Larger buffers (depth=10) help with delivery guarantees

### Migration Guide

**From Legacy to Threaded Architecture**:
1. Replace `test_camera_node` with `test_camera_threaded_node`
2. Update QoS settings based on use case (RELIABLE for display, BEST_EFFORT for AI)
3. Use `test_camera_ui` package for Qt-based display instead of OpenCV
4. Test with both laptop camera (index 0) and external USB camera (index 4)

## See Also
- [`test_camera_ui` package](../test_camera_ui/README.md): Optimized Qt-based camera display
- [`human_interaction` package](../human_interaction/README.md): AI processing and gesture recognition
- [Robot Dog Petting Zoo Documentation](../../PHASE1_IMPLEMENTATION_README.md): Complete system overview