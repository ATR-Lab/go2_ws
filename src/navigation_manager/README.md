# Navigation Manager

Ultra-simple Nav2 coordinator for human-aware navigation in the robot dog petting zoo. This package provides a lightweight bridge between human interaction detection and Nav2 navigation, enabling safe and responsive robot behavior around humans.

## Overview

The Navigation Manager acts as a coordinator between the `human_interaction` system and Nav2, providing:

- **Human-Aware Navigation**: Automatically pauses navigation when humans are detected
- **Step Back Behaviors**: Uses Nav2's BackUp action for safe proximity responses  
- **Goal Management**: Stores and resumes navigation goals after human interactions
- **External Goal Interface**: Ready for iPad interface and patrol system integration

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    NAVIGATION COORDINATOR                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  Human Detection → Presence Tracking → Navigation Control      │
│                                                                 │
│  ┌─────────────────┐    ┌──────────────┐    ┌───────────────┐  │
│  │ Human Presence  │───→│ Goal Storage │───→│ Nav2 Actions  │  │
│  │ - Camera based  │    │ - Pause/Resume│    │ - NavigateToPose│  │
│  │ - Timeout logic │    │ - External goals│   │ - BackUp      │  │
│  └─────────────────┘    └──────────────┘    └───────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                                │
                                ▼
                    ┌─────────────────────────┐
                    │ Nav2 Stack              │
                    │ - BT Navigator          │
                    │ - Controller Server     │
                    │ - Behavior Server       │
                    └─────────────────────────┘
```

## Key Features

### **Ultra-Simple Design**
- **No Complex State Machines**: Lets Nav2 handle navigation complexity
- **Minimal Parameters**: Only essential configuration needed
- **Direct Action Calls**: Simple translation of events to Nav2 actions

### **Human-Aware Behavior**
- **Automatic Pause**: Cancels navigation goals when humans detected
- **Timeout Logic**: 5-second delay before considering humans "gone"
- **Resume Delay**: 2-second buffer before resuming navigation
- **Step Back Safety**: Uses Nav2 BackUp action for proximity responses

### **External Integration Ready**
- **iPad Interface**: Accepts goals via `/goal_pose` topic
- **Patrol System**: Can integrate with waypoint following
- **Blockchain Events**: Ready for NFT treat integration

## Data Flow

### **Input Topics**
- `/interaction/nav_commands` → Navigation commands from interaction_manager
- `/interaction/events` → Human presence events (entered/left area)  
- `/human_detection/people` → Direct human detection for presence tracking
- `/goal_pose` → External navigation goals (iPad, patrol system)

### **Output Actions**
- `navigate_to_pose` → Nav2 NavigateToPose action client
- `backup` → Nav2 BackUp action client

### **Navigation Logic**
```
Human Detected → Cancel Current Goal → Store Goal → Wait for Human to Leave
Human Left (5s timeout) → Resume Stored Goal (2s delay) → Navigate
Step Back Command → Execute BackUp Action → Return to Previous State
```

## Configuration

### **Parameters (`config/navigation_params.yaml`)**

```yaml
navigation_coordinator:
  ros__parameters:
    # Step back behavior
    step_back_distance: 0.5    # meters to step back
    step_back_speed: 0.2       # m/s stepping speed
    
    # Human presence detection
    human_timeout: 5.0         # seconds before "human gone"
    resume_delay: 2.0          # seconds delay before resuming
```

### **Parameter Tuning**
- **`step_back_distance`**: Increase for more conservative safety (0.3-1.0m)
- **`step_back_speed`**: Decrease for gentler movement (0.1-0.5m/s)
- **`human_timeout`**: Increase to reduce flicker on tracking failures (3-10s)
- **`resume_delay`**: Increase for more conservative resumption (1-5s)

## Usage

### **Launch Navigation Manager**
```bash
# Launch with default parameters
ros2 launch navigation_manager navigation_manager.launch.py

# Launch with custom parameters
ros2 launch navigation_manager navigation_manager.launch.py \
    params_file:=/path/to/custom_params.yaml \
    log_level:=debug
```

### **Integration with Robot System**
```bash
# 1. Launch main robot system (includes Nav2)
ros2 launch go2_robot_sdk robot.launch.py

# 2. Launch human interaction system
ros2 launch human_interaction human_interaction.launch.py

# 3. Launch navigation manager
ros2 launch navigation_manager navigation_manager.launch.py
```

### **Send Navigation Goals**
```bash
# Send goal via command line (iPad interface simulation)
ros2 topic pub /goal_pose geometry_msgs/PoseStamped '{
  header: {frame_id: "map"},
  pose: {
    position: {x: 2.0, y: 1.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}'
```

### **Monitor Navigation Status**
```bash
# Monitor human presence detection
ros2 topic echo /human_detection/people

# Monitor navigation commands
ros2 topic echo /interaction/nav_commands

# Monitor interaction events
ros2 topic echo /interaction/events
```

## Integration Points

### **With Human Interaction Package**
- **Subscribes to**: `/interaction/nav_commands` for step_back commands
- **Subscribes to**: `/interaction/events` for human presence events
- **Subscribes to**: `/human_detection/people` for direct presence tracking

### **With Nav2 Stack**
- **Uses**: NavigateToPose action client for goal navigation
- **Uses**: BackUp action client for step_back behaviors
- **Integrates**: Goal cancellation and resumption with Nav2's BT Navigator

### **With External Systems**
- **iPad Interface**: Accepts goals via `/goal_pose` topic
- **Patrol System**: Can integrate with waypoint following nodes
- **Blockchain**: Ready for NFT treat event integration

## Troubleshooting

### **Navigation Not Pausing for Humans**
```bash
# Check human detection is working
ros2 topic echo /human_detection/people

# Check navigation coordinator is receiving data
ros2 node info /navigation_coordinator

# Verify topic connections
ros2 topic info /human_detection/people
```

### **Navigation Not Resuming After Humans Leave**
```bash
# Check timeout parameters
ros2 param get /navigation_coordinator human_timeout
ros2 param get /navigation_coordinator resume_delay

# Monitor human timeout logic
ros2 topic echo /interaction/events
```

### **Step Back Not Working**
```bash
# Check Nav2 BackUp action server
ros2 action list | grep backup

# Test BackUp action directly
ros2 action send_goal /backup nav2_msgs/action/BackUp "{target: 0.5, speed: 0.2}"

# Check step back parameters
ros2 param get /navigation_coordinator step_back_distance
```

### **Goals Not Being Accepted**
```bash
# Check Nav2 NavigateToPose action server
ros2 action list | grep navigate_to_pose

# Verify goal format
ros2 interface show geometry_msgs/PoseStamped

# Check coordinate frames
ros2 run tf2_tools view_frames.py
```

## Development

### **Adding New Navigation Behaviors**
1. **Add command to interaction_manager**: Update `UNIFIED_COMMANDS` dictionary
2. **Handle in navigation_coordinator**: Add case in `nav_command_callback`
3. **Implement Nav2 integration**: Use appropriate Nav2 action client

### **Extending Human Presence Detection**
1. **Modify timeout logic**: Adjust `check_human_timeout` method
2. **Add presence confidence**: Enhance `people_callback` with confidence thresholds
3. **Multi-zone detection**: Add zone-based presence logic

### **iPad Interface Integration**
1. **Goal format**: Use standard `geometry_msgs/PoseStamped` on `/goal_pose`
2. **Status feedback**: Monitor Nav2 action feedback for goal status
3. **Error handling**: Implement goal validation and error reporting

## Dependencies

### **ROS2 Packages**
- `rclpy` - ROS2 Python client library
- `nav2_msgs` - Nav2 action messages (NavigateToPose, BackUp)
- `geometry_msgs` - Pose and spatial messages
- `std_msgs` - String messages for interaction events
- `action_msgs` - Action client/server support

### **System Requirements**
- **Nav2 Stack**: Must be running with NavigateToPose and BackUp action servers
- **Human Detection**: Requires `human_interaction` package for presence detection
- **Transform Tree**: Proper TF setup for coordinate frame conversions

## License

BSD-3-Clause - See LICENSE file for details.

## Authors

Robot Dog Petting Zoo Team