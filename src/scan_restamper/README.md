# Scan Restamper Package

## Overview

The `scan_restamper` package provides ROS2 nodes for handling timing and frame issues in laser scan data, particularly useful for WebRTC-based robot systems like the Unitree Go2.

## Purpose

This package addresses two critical issues in ROS2 navigation systems:

1. **Timing Issues**: Restamps laser scan messages with current time to fix timing problems in Nav2
2. **Frame Mismatch**: Converts sensor data between different coordinate frames (e.g., `base_link` to `base_footprint`)

## Components

### 1. Scan Restamper Node (`scan_restamper_node`)

**Purpose**: Restamps laser scan messages and optionally changes the frame ID.

**Key Features**:
- Restamps sensor data with current time to fix WebRTC timing issues
- Can override the frame_id to resolve coordinate system mismatches
- Uses best-effort QoS for sensor data reliability
- Configurable input/output topics

**Parameters**:
- `input_topic` (string, default: `/scan`): Input laser scan topic
- `output_topic` (string, default: `/scan_restamped`): Output laser scan topic
- `frame_id` (string, default: `""`): Frame ID override (empty = keep original)

### 2. Navigation Action Restamper (`restamp_nav_actions`)

**Purpose**: Restamps navigation action messages to handle timing issues in action servers.

**Features**:
- Relays `NavigateToPose` actions with restamped timestamps
- Relays `NavigateThroughPoses` actions with restamped timestamps
- Handles goal, cancel, and feedback callbacks
- Maintains proper action server/client relationships

## Usage

### Option 1: Launch File Integration (Recommended)

The scan restamper is integrated into the main robot launch file:

```bash
ros2 launch go2_robot_sdk robot.launch.py
```

This automatically starts the scan restamper with the correct configuration:
- Input: `/scan` (sensor data in `base_link` frame)
- Output: `/scan_restamped` (data in `base_footprint` frame)
- Frame override: `base_footprint`

### Option 2: Standalone Node

Run the scan restamper as a separate node:

```bash
# Basic usage (keeps original frame)
ros2 run scan_restamper scan_restamper_node

# With frame override (recommended for Go2)
ros2 run scan_restamper scan_restamper_node --ros-args -p frame_id:=base_footprint

# With custom topics
ros2 run scan_restamper scan_restamper_node --ros-args \
  -p input_topic:=/scan \
  -p output_topic:=/scan_restamped \
  -p frame_id:=base_footprint
```

### Option 3: Navigation Action Restamper

For navigation action timing issues:

```bash
ros2 run scan_restamper restamp_nav_actions
```

## Configuration

### Frame ID Configuration

**Problem**: Sensor data published in `base_link` frame, but navigation expects `base_footprint` frame.

**Solution**: Configure the scan restamper to output `base_footprint`:

```yaml
scan_restamper_node:
  ros__parameters:
    input_topic: "/scan"
    output_topic: "/scan_restamped"
    frame_id: "base_footprint"
```

### Topics

**Input Topic**: `/scan`
- Source: LiDAR/radar sensor data
- Frame: `base_link` (original sensor frame)

**Output Topic**: `/scan_restamped`
- Destination: SLAM and Nav2 components
- Frame: `base_footprint` (navigation frame)

## Integration with Go2 Robot

### Frame Hierarchy

The Go2 robot uses this coordinate frame hierarchy:
```
map → odom → base_footprint → base_link → sensor_link
```

### Why Frame Conversion is Needed

1. **Sensors publish in `base_link`**: LiDAR/radar data comes in robot body frame
2. **Navigation expects `base_footprint`**: Nav2 and SLAM work in ground-level frame
3. **Transform issues**: WebRTC latency can cause transform lookup failures
4. **Solution**: Scan restamper converts frames to eliminate transform dependencies

### Benefits

- **Eliminates frame mismatch errors**
- **Reduces WebRTC timing issues**
- **Enables real-time obstacle detection**
- **Improves navigation reliability**
- **Simplifies coordinate system management**

## Troubleshooting

### Common Issues

1. **Message Filter Queue Overflow**
   ```
   Message Filter dropping message: frame 'base_link' for reason 'queue is full'
   ```
   **Solution**: Ensure scan restamper is running with correct frame_id parameter

2. **Sensor Origin Out of Bounds**
   ```
   Sensor origin at (-11.60, -9.73 -0.01) is out of map bounds
   ```
   **Solution**: Frame mismatch - configure scan restamper to output `base_footprint`

3. **No Real-time Obstacle Detection**
   **Solution**: Verify scan restamper is converting frames correctly

### Verification

Check that the scan restamper is working:

```bash
# Check input topic frame
ros2 topic echo /scan --once | grep frame_id

# Check output topic frame
ros2 topic echo /scan_restamped --once | grep frame_id

# Verify transform tree
ros2 run tf2_tools view_frames
```

## Dependencies

- ROS2 Humble
- sensor_msgs
- nav2_msgs
- rclpy

## License

BSD-3-Clause
