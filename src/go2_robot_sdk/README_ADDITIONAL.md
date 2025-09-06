# Go2 Robot SDK - Collision Monitor Setup

This document provides instructions for launching the Go2 robot system with collision monitoring capabilities.

## System Overview

The Go2 robot system consists of two main components:
1. **Main Robot Stack** - Navigation, SLAM, control, and sensors
2. **Collision Monitor** - Safety layer for obstacle avoidance (now integrated)

## Quick Start

### Launch Complete Robot System with Collision Monitor

Launch the complete robot navigation stack with integrated collision monitoring:

```bash
cd ~/Projects/ROS/go2_ws
ros2 launch go2_robot_sdk robot.launch.py
```

This starts:
- Go2 driver node (WebRTC connection)
- Nav2 navigation stack
- SLAM toolbox
- Velocity smoother
- Scan restamper (converts `/scan` to `/scan_restamped`)
- **Collision monitor** (safety layer for obstacle avoidance)
- RViz visualization

### Optional: Launch Without Collision Monitor

If you need to run without collision monitoring:

```bash
cd ~/Projects/ROS/go2_ws
ros2 launch go2_robot_sdk robot.launch.py collision_monitor:=false
```

## System Architecture

### Topic Flow
```
Nav2 Controller → cmd_vel_nav → Collision Monitor → cmd_vel → Robot
                                      ↑
                               scan_restamped (sensor input)
```

### Key Topics
- **Input to Collision Monitor**: `/cmd_vel_nav`, `/scan_restamped`
- **Output from Collision Monitor**: `/cmd_vel`, `/collision_monitor_state`
- **Visualization**: `/polygon_stop`, `/polygon_approach`

## Verification

### Check Collision Monitor Status
```bash
# Verify collision monitor is active
ros2 node info /collision_monitor

# Check collision monitor state
ros2 topic echo /collision_monitor_state

# Monitor collision polygons in RViz
# Add visualization for /polygon_stop and /polygon_approach topics
```

### Test Collision Detection
1. Set a navigation goal in RViz
2. Place an obstacle in the robot's path
3. Observe collision monitor stopping or slowing the robot
4. Check velocity commands: `ros2 topic echo /cmd_vel`

## Configuration Files

- **`config/nav2_params.yaml`** - Main Nav2 configuration
- **`config/collision_monitor_params.yaml`** - Collision monitor specific settings
- **`launch/robot.launch.py`** - Main robot launch file (now includes collision monitor)

## Collision Monitor Configuration

The collision monitor is configured with two safety zones:

### FootprintStop (Immediate Stop Zone)
- **Dimensions**: 0.81m x 0.40m (Go2 physical footprint)
- **Action**: Immediate stop when obstacles detected
- **Time horizon**: 0.2 seconds

### FootprintApproach (Early Warning Zone)  
- **Dimensions**: 0.90m x 0.50m (with approach margins)
- **Action**: Immediate stop for early obstacle detection
- **Time horizon**: 0.3 seconds

## Environment Variables

Ensure these environment variables are set:

```bash
export ROBOT_TOKEN="your_robot_token"
export ROBOT_IP="192.168.1.100"  # Your robot's IP
export CONN_TYPE="webrtc"
```

## Troubleshooting

### Collision Monitor Not Starting

1. **Check if collision monitor is enabled**:
   ```bash
   # Verify collision monitor node is running
   ros2 node list | grep collision_monitor
   ```

2. **Verify scan_restamper is running**:
   ```bash
   ros2 topic list | grep scan_restamped
   ```

3. **Check for YAML syntax errors**:
   ```bash
   python3 -c "import yaml; print('✅ YAML valid' if yaml.safe_load(open('src/go2_robot_sdk/config/collision_monitor_params.yaml')) else '❌ YAML invalid')"
   ```

### No Collision Detection

1. **Verify topic connections**:
   ```bash
   ros2 topic echo /cmd_vel_nav  # Should show navigation commands
   ros2 topic echo /scan_restamped  # Should show laser data
   ```

2. **Check collision polygons in RViz**:
   - Add `geometry_msgs/PolygonStamped` visualization
   - Subscribe to `/polygon_stop` topic

3. **Monitor collision monitor state**:
   ```bash
   ros2 topic echo /collision_monitor_state
   ```

## RViz Configuration

Add these visualization topics in RViz:
- **Robot footprint**: `/local_costmap/published_footprint`
- **Collision zones**: `/polygon_stop`, `/polygon_approach`  
- **Laser scan**: `/scan_restamped`
- **Navigation path**: `/plan`

## Safety Notes

⚠️ **Important**: The collision monitor acts as a safety layer but should not be the only obstacle avoidance mechanism. Always:

- Ensure proper lighting for lidar operation
- Test in safe, controlled environments first
- Monitor robot behavior during initial deployments
- Have emergency stop capabilities ready

## Support

For issues or questions:
1. Check the logs in `~/.ros/log/`
2. Verify all required topics are publishing
3. Test individual components separately
4. Review Nav2 and collision monitor documentation

---

**Last Updated**: January 2025
**Compatible with**: ROS2 Humble, Nav2 1.1.x
