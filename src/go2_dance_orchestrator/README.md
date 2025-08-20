# Go2 Dance Orchestrator

ROS2 package for orchestrating dance routines on Go2 robot with command completion tracking.

## Overview

This package provides a comprehensive dance orchestration system for the Unitree Go2 robot, featuring:

- **Single Command Execution**: Execute individual dance moves with completion tracking
- **Hybrid Completion Detection**: Intelligent robot state monitoring with intelligent timing fallback
- **Command-Specific Timing**: Each dance move has appropriate expected duration
- **Automatic Upgrade**: Seamlessly switches to robot feedback when available
- **ROS2 Integration**: Clean integration with existing `go2_robot_sdk`
- **Extensible Architecture**: Designed for easy addition of dance sequences and new commands

## Features

### Phase 1: Single Command Tracker ✅

- Execute individual dance commands: "Hello", "FrontFlip", "Dance1", etc.
- **Hybrid completion detection** with two complementary approaches:
  - **Primary**: Robot state feedback (progress, mode, movement analysis)
  - **Fallback**: Intelligent command-specific timing (works without robot state)
- **Automatic detection mode switching**: Uses robot feedback when available, timing when not
- **Smooth progress indication**: Realistic progress curves based on command type
- **Real-time status publishing**: Progress, timing, and completion reasons
- **Simple service interface**: No configuration required - just specify command name

## Installation

```bash
# Build the packages
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select go2_interfaces go2_dance_orchestrator
source install/setup.bash
```

## Usage

### Start the Dance Orchestrator

```bash
# Terminal 1: Start the go2_robot_sdk (must be running first)
ros2 launch go2_robot_sdk robot.launch.py

# Terminal 2: Start the dance orchestrator
ros2 launch go2_dance_orchestrator single_command_test.launch.py
```

### Execute Single Dance Commands

```bash
# Execute a greeting gesture (completes in ~3 seconds)
ros2 service call /execute_dance_command go2_interfaces/srv/ExecuteSingleCommand "{command_name: 'Hello'}"

# Execute a front flip (completes in ~5 seconds)  
ros2 service call /execute_dance_command go2_interfaces/srv/ExecuteSingleCommand "{command_name: 'FrontFlip'}"

# Execute built-in dance routine (completes in ~10 seconds)
ros2 service call /execute_dance_command go2_interfaces/srv/ExecuteSingleCommand "{command_name: 'Dance1'}"
```

### Monitor Command Status

```bash
# Monitor real-time status updates
ros2 topic echo /dance_command_status
```

### Stop Current Command

```bash
# Emergency stop current command
ros2 service call /stop_dance_command go2_interfaces/srv/StopDanceRoutine "{}"
```

## Available Commands

| Command | Duration | Description | Category |
|---------|----------|-------------|----------|
| `Hello` | 3.0s | Friendly greeting gesture | greeting |
| `FrontFlip` | 5.0s | Forward flip maneuver | acrobatic |
| `Dance1` | 10.0s | Built-in dance routine #1 | dance |
| `Dance2` | 8.0s | Built-in dance routine #2 | dance |
| `Handstand` | 6.0s | Handstand pose | acrobatic |
| `WiggleHips` | 4.0s | Hip wiggling dance move | dance |
| `FingerHeart` | 3.5s | Finger heart gesture | gesture |
| `MoonWalk` | 7.0s | Moonwalk dance move | dance |
| `Stretch` | 4.0s | Stretching routine | exercise |
| `Pose` | 2.0s | Strike a pose | pose |

## Testing

Run the included test script to verify functionality:

```bash
# Run automated test sequence
python3 src/go2_dance_orchestrator/test/test_single_command.py
```

## Architecture

```
go2_dance_orchestrator/
├── domain/                     # Business logic
│   ├── entities/              # CommandExecution, DanceSequence
│   ├── interfaces/            # ICommandTracker, ICompletionDetector
│   └── enums/                 # CommandStatus, CompletionReason
├── application/               # Application services
│   └── services/             # SingleCommandTracker, CompletionDetector
├── infrastructure/           # External integrations
│   └── integration/         # Go2SDKBridge
└── presentation/            # ROS2 interface
    └── dance_orchestrator_node.py
```

## Message Types

### CommandExecutionStatus
Real-time status of command execution with progress, timing, and completion reason.

### Services

- `ExecuteSingleCommand`: Execute a single dance command
- `StopDanceRoutine`: Stop currently executing command

## Completion Detection

The system uses a **hybrid approach** with automatic fallback for robust completion detection:

### Primary: Robot State Feedback (When Available)
1. **Progress Monitoring**: Tracks robot progress field returning to baseline
2. **Mode Detection**: Monitors robot mode changes back to idle  
3. **Movement Stillness**: Detects when robot velocity drops to near zero

### Fallback: Intelligent Timing (Always Works)
- **Command-specific durations**: Each command has realistic expected completion time
- **Smart progress curves**: Natural progress indication even without robot state
- **Immediate availability**: Works even when robot state data is unavailable

### Automatic Mode Selection
- **Attempts robot state detection first** (3-second grace period)
- **Seamlessly falls back to timing** if no robot state data flows
- **Automatically upgrades to robot state** if data becomes available later
- **No configuration needed**: Works out-of-the-box in any environment

## Configuration

The hybrid detection system is **zero-configuration** by design:

- **Built-in command timing**: All commands have pre-tuned intelligent durations
- **Automatic detection**: No setup needed for robot state vs. timing modes  
- **Self-adapting thresholds**: Detection parameters optimized for reliable completion
- **Override capability**: Advanced users can modify detection parameters via ROS2 parameters if needed

### Command Duration Mapping
```python
# Built-in intelligent durations (seconds)
"Hello": 3.0          # Quick greeting
"Dance1": 10.0        # Full dance routine  
"FrontFlip": 5.0      # Athletic move
"WiggleHips": 6.0     # Expressive dance
# ... and more
```

## Why Hybrid Detection?

This approach provides the **best of both worlds**:

### ✅ **Immediate Functionality**
- Works instantly, even without robot state data flowing
- No debugging WebRTC sensor streams required  
- Reliable for development, testing, and demos

### ✅ **Future-Ready**
- Automatically upgrades when robot state becomes available
- No code changes needed to benefit from robot feedback
- Preserves pure detection architecture for optimal performance

### ✅ **Robust Operation** 
- **Network resilient**: Not dependent on continuous data streams
- **Condition adaptive**: Works regardless of robot state availability
- **Graceful degradation**: Falls back smoothly when robot connection issues occur

### ✅ **User-Friendly**
- **Zero configuration**: Works out-of-the-box
- **Predictable timing**: Users know approximately when commands complete
- **Clear feedback**: Completion reasons indicate which detection method was used

## Future Enhancements (Roadmap)

### Phase 2: Dance Sequence Engine
- Execute pre-defined dance routines with multiple commands
- Sequence-level progress tracking and coordination
- YAML-based routine definitions

### Phase 3: Advanced Features  
- Dynamic routine loading and validation
- Performance analytics and logging
- Web/UI integration capabilities

## Dependencies

- `go2_robot_sdk`: Main robot control SDK
- `go2_interfaces`: Message and service definitions
- `numpy`: Mathematical operations for detection algorithms

## Troubleshooting

### Common Issues

1. **"Service not available"**
   - Ensure `go2_robot_sdk` is running first
   - Check that all packages are built and sourced

2. **Commands not completing**
   - Monitor `/dance_command_status` for completion reasons
   - Check robot connection and WebRTC status
   - Verify robot is in appropriate mode for dance commands

3. **Build errors**
   - Ensure all dependencies are installed
   - Clean build with `rm -rf build install` and rebuild

## Contributing

This package follows clean architecture principles. When adding new features:

1. Add domain entities in `domain/entities/`
2. Define interfaces in `domain/interfaces/`
3. Implement services in `application/services/`
4. Update ROS2 integration in `presentation/`

## License

Apache-2.0