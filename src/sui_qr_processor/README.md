# Sui QR Processor

A ROS2 package that processes QR codes from camera feeds and executes Sui blockchain transactions with comprehensive duplicate prevention.

## Overview

This package subscribes to a camera image topic, detects QR codes containing Sui transaction data, and automatically executes the transactions on the Sui network. It includes robust duplicate prevention to ensure transactions are not executed multiple times.

## Features

- **QR Code Detection**: Real-time QR code detection from camera feed using pyzbar
- **Sui Integration**: Execute transactions on Sui blockchain using sui-py
- **Duplicate Prevention**: Multi-layered system to prevent duplicate transaction execution
- **Network Configuration**: Support for testnet, mainnet, and devnet with flexible configuration
- **Cooldown System**: Configurable cooldown period between transaction executions
- **ROS2 Integration**: Full ROS2 node with parameters, services, and publishers

## Dependencies

### System Dependencies
```bash
# Install pyzbar dependencies
sudo apt-get install libzbar0

# Install Python packages
pip install opencv-python pyzbar sui-py
```

### ROS2 Dependencies
- `rclpy`
- `sensor_msgs`
- `geometry_msgs`
- `cv_bridge`

## QR Code Format

The QR codes must contain JSON data with the following structure:

```json
{
  "bytes": "base64_encoded_transaction_bytes",
  "signature": "base64_encoded_signature",
  "chain": "sui:network_name"
}
```

Where:
- `bytes`: Base64-encoded Sui transaction bytes
- `signature`: Base64-encoded transaction signature
- `chain`: Network identifier (e.g., "sui:testnet", "sui:mainnet")

## Configuration

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `camera_topic` | `/camera/image_raw` | Camera image topic to subscribe to |
| `default_network` | `testnet` | Default Sui network |
| `allowed_networks` | `["testnet", "mainnet", "devnet"]` | Allowed networks for security |
| `network_source` | `qr_with_fallback` | Network selection strategy |
| `qr_detection_interval` | `0.2` | QR detection processing interval (seconds) |
| `execution_cooldown` | `30.0` | Minimum time between executions (seconds) |
| `cache_max_size` | `1000` | Maximum cached transactions |

### Network Selection Strategies

- `qr_with_fallback`: Use QR network if valid, otherwise use default
- `config_only`: Always use default network, ignore QR chain field
- `qr_only`: Only use QR network, fail if invalid

## Usage

### Basic Launch
```bash
ros2 launch sui_qr_processor sui_qr_processor.launch.py
```

### Custom Configuration
```bash
# Use mainnet
ros2 launch sui_qr_processor sui_qr_processor.launch.py network:=mainnet

# Custom camera topic
ros2 launch sui_qr_processor sui_qr_processor.launch.py camera_topic:=/my_camera/image_raw

# Config-only network mode (ignore QR network)
ros2 launch sui_qr_processor sui_qr_processor.launch.py network_source:=config_only

# Debug logging
ros2 launch sui_qr_processor sui_qr_processor.launch.py log_level:=debug
```

### Direct Node Execution
```bash
ros2 run sui_qr_processor sui_qr_processor_node
```

## Topics

### Subscribed Topics
- `~/camera/image_raw` (sensor_msgs/Image): Camera image feed

### Published Topics
- `~/transaction_status` (std_msgs/String): Transaction execution status
- `~/qr_detection_status` (std_msgs/String): QR code detection events

### Services
- `~/reset_cache` (std_srvs/SetBool): Reset transaction cache

## Building

```bash
# Navigate to workspace
cd /path/to/your/ros2_ws

# Build the package
colcon build --packages-select sui_qr_processor

# Source the workspace
source install/setup.bash
```

## Testing

### Check Node Status
```bash
ros2 node info /sui_qr_processor_node
```

### Monitor Topics
```bash
# Monitor transaction status
ros2 topic echo /sui_qr_processor_node/transaction_status

# Monitor QR detection
ros2 topic echo /sui_qr_processor_node/qr_detection_status
```

### Reset Cache
```bash
ros2 service call /sui_qr_processor_node/reset_cache std_srvs/srv/SetBool "{data: true}"
```

## Duplicate Prevention

The system implements multiple layers of duplicate prevention:

1. **Transaction Hash Cache**: SHA-256 hash of QR content
2. **Processing Queue**: Prevents concurrent processing of same QR
3. **Execution Cooldown**: Minimum time between any executions
4. **Success Tracking**: Remembers successful transaction digests
5. **Automatic Cleanup**: Removes old entries to prevent memory growth

## Security Considerations

- **Network Allowlist**: Only configured networks are allowed
- **Input Validation**: QR data is thoroughly validated before processing
- **Error Handling**: Comprehensive error handling prevents crashes
- **Cooldown Protection**: Prevents rapid-fire transaction execution

## Troubleshooting

### Common Issues

1. **"pyzbar not available"**: Install libzbar0 and pyzbar
2. **"sui-py not available"**: Install sui-py package
3. **"QR network not in allowed list"**: Check allowed_networks parameter
4. **"In cooldown period"**: Wait for cooldown or adjust execution_cooldown parameter

### Debug Mode
```bash
ros2 launch sui_qr_processor sui_qr_processor.launch.py log_level:=debug
```

### Check Dependencies
```bash
python3 -c "import pyzbar; print('pyzbar OK')"
python3 -c "import sui_py; print('sui-py OK')"
python3 -c "import cv2; print('opencv OK')"
```

## License

MIT License
