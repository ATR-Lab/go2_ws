# Go2 Robot UI

A comprehensive PyQt-based user interface for controlling and monitoring Unitree Go2 robots via ROS2. This package provides real-time camera display with QR code detection, intuitive robot command controls, and comprehensive status monitoring.

## Features

### 🎥 Camera Display
- **Real-time Video Stream**: Live camera feed from Go2 robot's front camera
- **QR Code Detection**: Automatic QR code detection with visual overlay
- **Performance Metrics**: FPS, latency, and connection status monitoring
- **Auto-scaling**: Video automatically fits display area

### 🎮 Robot Control
- **Predefined Actions**: 25+ robot actions organized by category
  - **Basic**: Stand, Sit, Hello, Stretch, Balance
  - **Entertainment**: Dance1, Dance2, WiggleHips, FingerHeart
  - **Athletic**: FrontFlip, Handstand, MoonWalk, Bound
  - **Emergency**: StopMove for immediate halt
- **QR Code Commands**: Execute actions by showing QR codes to camera
- **Visual Feedback**: Button highlighting and command status display

### 📊 Status Monitoring
- **Robot State**: Mode, gait, position, velocity, body height
- **IMU Sensor**: Orientation, angular velocity, acceleration
- **Joint States**: Motor positions, velocities, and efforts
- **Connection Status**: Real-time monitoring of robot communication

### 🔧 Architecture
- **Modular Design**: Separable widgets and ROS interface components
- **Thread-safe**: Proper Qt/ROS2 integration with 1ms update cycle
- **Performance Optimized**: Efficient video processing and UI updates
- **Extensible**: Easy to add new commands and features

## Installation

### Prerequisites
- ROS2 Humble/Iron
- Python 3.10+
- Qt5/Qt6 development packages
- OpenCV Python bindings

### Dependencies
The package requires these ROS2 packages:
- `go2_robot_sdk` (for robot communication)
- `go2_interfaces` (for message definitions)
- `cv_bridge`, `python_qt_binding`
- `sensor_msgs`, `geometry_msgs`, `std_msgs`

Python packages (auto-installed):
- `opencv-python`, `numpy`, `pyzbar`

### Building
```bash
# Navigate to your workspace
cd /path/to/ros2_ws

# Build the package
colcon build --packages-select go2_robot_ui

# Source the workspace
source install/setup.bash
```

## Usage

### Quick Start

1. **Set up robot connection** (ensure Go2 is connected):
```bash
export ROBOT_IP="192.168.1.100"  # Your robot's IP
export CONN_TYPE="webrtc"
```

2. **Launch complete system** (robot SDK + UI):
```bash
ros2 launch go2_robot_ui go2_full_system.launch.py
```

3. **Or launch UI only** (if robot SDK is already running):
```bash
ros2 launch go2_robot_ui go2_robot_ui.launch.py
```

### Launch Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `camera_topic` | `/camera/image_raw` | Camera video topic |
| `webrtc_topic` | `/webrtc_req` | Robot command topic |
| `robot_namespace` | `""` | Namespace for multi-robot |
| `log_level` | `info` | Logging verbosity |

### QR Code Commands

The UI supports QR code-triggered actions. Create QR codes with these values:

| QR Code | Action | Description |
|---------|--------|-------------|
| `HELLO` | Hello | Wave gesture |
| `SIT` | Sit | Sit down |
| `STAND` | StandUp | Stand up |
| `DANCE` | Dance1 | Dance routine |
| `STRETCH` | Stretch | Stretching |
| `FLIP` | FrontFlip | Front flip |
| `STOP` | StopMove | Emergency stop |

### Multi-Robot Support

For multi-robot setups, use namespaced topics:
```bash
ros2 launch go2_robot_ui go2_robot_ui.launch.py \
    camera_topic:=/robot0/camera/image_raw \
    webrtc_topic:=/robot0/webrtc_req \
    robot_namespace:=robot0
```

## Interface Layout

```
┌─────────────────────────────────────────────────────────────────┐
│                Go2 Robot Control Interface                     │
├─────────────────────────────────┬───────────────────────────────┤
│                                 │      Command Panel           │
│                                 │ ┌───────────────────────────┐ │
│                                 │ │    Basic Actions          │ │
│        Camera Display           │ │ [Stand Up] [Sit] [Hello]  │ │
│     (Video + QR Overlay)        │ │                           │ │
│                                 │ │    Entertainment          │ │
│                                 │ │ [Dance 1] [Dance 2]       │ │
│                                 │ │                           │ │
├─────────────────────────────────┤ │    Athletic Actions       │ │
│        Status Panel             │ │ [Front Flip] [Handstand]  │ │
│  Robot State | IMU | Joints     │ │                           │ │
│  Connection: ●  Mode: Sport     │ │    Emergency              │ │
│  Position: X:1.2 Y:0.5 Z:0.0    │ │ [STOP MOVE]              │ │
└─────────────────────────────────┴───────────────────────────────┘
```

## ROS Topics

### Subscribed Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Camera video stream |
| `/go2_states` | `go2_interfaces/Go2State` | Robot state data |
| `/imu` | `go2_interfaces/IMU` | IMU sensor data |
| `/joint_states` | `sensor_msgs/JointState` | Motor states |

### Published Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/webrtc_req` | `go2_interfaces/WebRtcReq` | Robot commands |

## Development

### Adding New Commands
1. Add command to `ROBOT_COMMANDS` in `robot_controller.py`
2. Add button to appropriate category in `command_panel.py`
3. Update QR mapping if desired

### Adding New Widgets
1. Create widget in `go2_robot_ui/widgets/`
2. Add to `widgets/__init__.py`
3. Integrate in `go2_ui_node.py`

### Testing
```bash
# Build and test
colcon build --packages-select go2_robot_ui
source install/setup.bash

# Test with mock data
ros2 run go2_robot_ui go2_robot_ui
```

## Troubleshooting

### Common Issues

#### UI doesn't start
```bash
# Check Qt installation
python3 -c "from python_qt_binding.QtWidgets import QApplication; print('Qt OK')"

# Check ROS environment
echo $ROS_DISTRO
```

#### No camera feed
```bash
# Check camera topic
ros2 topic list | grep camera
ros2 topic hz /camera/image_raw

# Verify robot connection
ros2 topic echo /go2_states --once
```

#### Commands not working
```bash
# Check WebRTC topic
ros2 topic list | grep webrtc
ros2 topic echo /webrtc_req

# Test manual command
ros2 topic pub /webrtc_req go2_interfaces/msg/WebRtcReq \
  "{api_id: 1016, topic: 'rt/api/sport/request'}" --once
```

### Performance Tips
- Use wired connection for best video streaming
- Disable unnecessary RViz/Nav2 components when using UI
- Adjust QR detection interval for CPU optimization

## License

Apache-2.0

## Related Packages

- **[go2_robot_sdk](../go2_robot_sdk/)**: Core robot communication and control
- **[go2_interfaces](../go2_interfaces/)**: ROS message definitions
- **[coffee_vision_ui](../../development/coffee-buddy/coffee_ws/src/coffee_vision_ui/)**: UI architecture inspiration