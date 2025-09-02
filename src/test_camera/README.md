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
**Purpose**: USB camera capture and ROS2 publishing
- **Executable**: `ros2 run test_camera test_camera_node`
- **Publishes**: 
  - `/camera/image_raw` (sensor_msgs/Image): Camera frames
  - `/camera/camera_info` (sensor_msgs/CameraInfo): Camera calibration data
- **Parameters**:
  - `camera_index` (int, default: 0): USB camera device index
  - `frame_rate` (float, default: 30.0): Capture frame rate (Hz)
  - `image_width` (int, default: 640): Frame width (pixels)
  - `image_height` (int, default: 480): Frame height (pixels)
  - `enable_preview` (bool, default: true): Show OpenCV preview window

#### 2. `simple_camera_display`
**Purpose**: Enhanced camera display with AI overlays
- **Executable**: `ros2 run test_camera simple_camera_display`
- **Subscribes**:
  - `/camera/image_raw`: Camera feed
  - `/human_detection/people`: YOLO human detection results
  - `/human_detection/gestures`: MediaPipe gesture recognition
  - `/interaction/state`: Robot interaction state
- **Features**:
  - Real-time camera feed display
  - YOLO bounding boxes around detected humans
  - Gesture recognition text overlays
  - Interaction state display
  - Performance metrics (FPS, detection counts)
  - Screenshot capability (press 's')
- **Controls**:
  - `q`: Quit application
  - `s`: Save screenshot

#### 3. `detection_visualizer` (Advanced)
**Purpose**: PyQt5-based advanced visualization (may have Qt conflicts)
- **Executable**: `ros2 run test_camera detection_visualizer`
- **Note**: Uses `python_qt_binding` for ROS2 integration
- **Status**: May experience Qt platform conflicts on some systems

#### 4. `rviz_visualizer`
**Purpose**: RViz2 marker-based visualization
- **Executable**: `ros2 run test_camera rviz_visualizer`
- **Publishes**: `/visualization_marker_array` for RViz2 display
- **Usage**: Requires RViz2 with MarkerArray display configured

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

### Camera Display Features
- **Live Video**: Real-time camera feed from USB webcam
- **Human Detection**: Green bounding boxes around detected people
- **Gesture Recognition**: Text overlays showing recognized gestures:
  - `thumbs_up`, `pointing`, `peace_sign`, `rock_sign`
  - `open_hand`, `fist`, `ok_sign`, `wave`
  - `hands_visible` (when hands detected but no specific gesture)
- **Interaction State**: Top-right corner shows current robot state:
  - `patrol`, `human_detected`, `interaction`
- **Performance Metrics**: Bottom-left shows:
  - FPS counter
  - Human count
  - Gesture count

### Gesture Recognition
The system recognizes specific hand gestures:
- **👍 Thumbs Up**: Only thumb extended
- **👉 Pointing**: Index finger extended
- **✌️ Peace Sign**: Index and middle fingers
- **🤘 Rock Sign**: Index and pinky fingers
- **✋ Open Hand**: All fingers extended
- **✊ Fist**: No fingers extended
- **👌 OK Sign**: Thumb-index circle
- **👋 Wave**: Hand raised with fingers up

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

### Qt Display Issues
- Use `simple_camera_display` instead of `detection_visualizer`
- The simple display uses OpenCV and avoids Qt conflicts

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

### Testing Workflow
1. **Development**: Use `simple_camera_test.launch.py` for basic testing
2. **AI Testing**: Add human detection with second launch file
3. **Integration**: Test with full robot system
4. **Deployment**: Replace with actual robot camera

### Extending Functionality
- Add new gesture types in `human_interaction` package
- Modify display overlays in `simple_camera_display.py`
- Create custom launch configurations for specific test scenarios

## See Also
- [`human_interaction` package](../human_interaction/README.md): AI processing and gesture recognition
- [Robot Dog Petting Zoo Documentation](../../PHASE1_IMPLEMENTATION_README.md): Complete system overview