#!/bin/bash

# Robot Dog Petting Zoo System - Installation Script
# This script installs all necessary dependencies for the petting zoo system

set -e  # Exit on any error

echo "🤖 Robot Dog Petting Zoo System - Installation Script"
echo "=================================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if running as root
if [[ $EUID -eq 0 ]]; then
   print_error "This script should not be run as root"
   exit 1
fi

# Check if we're in a ROS2 workspace
if [ ! -f "src/CMakeLists.txt" ] && [ ! -d "src" ]; then
    print_error "This script must be run from the root of a ROS2 workspace"
    print_status "Please navigate to your ROS2 workspace directory and run this script"
    exit 1
fi

print_status "Starting installation of Robot Dog Petting Zoo dependencies..."

# Update package list
print_status "Updating package list..."
sudo apt update

# Install system dependencies
print_status "Installing system dependencies..."
sudo apt install -y \
    python3-opencv \
    python3-pip \
    python3-venv \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    unzip \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libgomp1

# Install ROS2 dependencies (if not already installed)
print_status "Installing ROS2 dependencies..."
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-nav-msgs \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-vision-opencv \
    ros-humble-rviz2 \
    ros-humble-nav2-* \
    ros-humble-slam-toolbox

# Create Python virtual environment (optional but recommended)
print_status "Setting up Python virtual environment..."
if [ ! -d "petting_zoo_venv" ]; then
    python3 -m venv petting_zoo_venv
    print_success "Created virtual environment: petting_zoo_venv"
else
    print_warning "Virtual environment already exists: petting_zoo_venv"
fi

# Activate virtual environment and install Python dependencies
print_status "Installing Python dependencies..."
source petting_zoo_venv/bin/activate

# Upgrade pip
pip install --upgrade pip

# Install Python packages from requirements.txt
if [ -f "requirements.txt" ]; then
    pip install -r requirements.txt
    print_success "Installed Python dependencies from requirements.txt"
else
    print_error "requirements.txt not found!"
    exit 1
fi

# Download YOLOv8 model
print_status "Downloading YOLOv8 model..."
mkdir -p models
cd models
if [ ! -f "yolov8n.pt" ]; then
    wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
    print_success "Downloaded YOLOv8n model"
else
    print_warning "YOLOv8n model already exists"
fi
cd ..

# Download MediaPipe Gesture Recognizer model
print_status "Downloading MediaPipe Gesture Recognizer model..."
mkdir -p models/mediapipe
cd models/mediapipe
if [ ! -f "gesture_recognizer.task" ]; then
    wget https://storage.googleapis.com/mediapipe-models/gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task
    print_success "Downloaded MediaPipe gesture recognizer model"
else
    print_warning "MediaPipe gesture recognizer model already exists"
fi
cd ../..

# Create necessary directories
print_status "Creating project directories..."
mkdir -p logs
mkdir -p config
mkdir -p data
mkdir -p maps
mkdir -p cache

# Set up environment variables
print_status "Setting up environment variables..."
if [ ! -f ".env" ]; then
    cat > .env << EOF
# Robot Dog Petting Zoo Environment Variables
export PETTING_ZOO_ROOT=\$(pwd)
export PYTHONPATH=\$PETTING_ZOO_ROOT/src:\$PYTHONPATH
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Optional: Set robot-specific variables
# export ROBOT_TOKEN="your_robot_token"
# export ROBOT_IP="192.168.1.100"
# export CONN_TYPE="webrtc"
EOF
    print_success "Created .env file"
else
    print_warning ".env file already exists"
fi

# Prevent colcon from scanning virtual environment
print_status "Configuring build system..."
if [ ! -f "petting_zoo_venv/COLCON_IGNORE" ]; then
    touch petting_zoo_venv/COLCON_IGNORE
    print_success "Added COLCON_IGNORE to virtual environment"
else
    print_warning "COLCON_IGNORE already exists in virtual environment"
fi

# Set up development tools
print_status "Setting up development tools..."

# Install pre-commit hooks (optional)
if command -v pre-commit &> /dev/null; then
    print_status "Installing pre-commit hooks..."
    pre-commit install
else
    print_warning "pre-commit not installed. Install with: pip install pre-commit"
fi

# Create a setup script for easy activation
cat > setup_petting_zoo.sh << 'EOF'
#!/bin/bash
# Quick setup script for Robot Dog Petting Zoo development

echo "🤖 Setting up Robot Dog Petting Zoo environment..."

# Source ROS2
source /opt/ros/humble/setup.bash

# Source workspace
source install/setup.bash

# Activate virtual environment
source petting_zoo_venv/bin/activate

# Set environment variables
if [ -f ".env" ]; then
    source .env
fi

echo "✅ Petting Zoo environment ready!"
echo "📦 Available packages:"
echo "   - human_interaction"
echo "   - test_camera" 
echo "   - go2_robot_sdk"
echo "   - speech_processor"
echo "   - navigation_manager"
echo "   - And more..."
echo ""
echo "🚀 To build: colcon build"
echo "🎯 To run: ros2 launch go2_robot_sdk robot.launch.py"
EOF

chmod +x setup_petting_zoo.sh
print_success "Created setup_petting_zoo.sh script"

# Test installation (run tests inline while venv is active)
print_status "Testing installation..."

# Test Python imports (in virtual environment)
print_status "Testing Python dependencies..."
python -c "
import cv2
import mediapipe
import ultralytics
import numpy as np
print('✅ All Python dependencies imported successfully')
" || {
    print_error "Python dependency test failed!"
    exit 1
}

# Test ROS2 imports (deactivate venv temporarily for ROS2 system packages)
deactivate
source /opt/ros/humble/setup.bash
print_status "Testing ROS2 dependencies..."
python3 -c "
import rclpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
print('✅ All ROS2 dependencies imported successfully')
" || {
    print_error "ROS2 dependency test failed!"
    exit 1
}

# Reactivate virtual environment
source petting_zoo_venv/bin/activate

# Test ROS2 packages exist
print_status "Checking ROS2 packages..."
if [ -d "src/human_interaction" ] && [ -d "src/test_camera" ] && [ -d "src/go2_robot_sdk" ]; then
    print_success "All required ROS2 packages found"
else
    print_error "Some ROS2 packages missing"
    exit 1
fi

# Test YOLO model
print_status "Checking YOLO model..."
if [ -f "models/yolov8n.pt" ]; then
    print_success "YOLOv8 model downloaded successfully"
else
    print_error "YOLOv8 model missing"
    exit 1
fi

# Test MediaPipe model
print_status "Checking MediaPipe model..."
if [ -f "models/mediapipe/gesture_recognizer.task" ]; then
    print_success "MediaPipe gesture recognizer model downloaded successfully"
else
    print_error "MediaPipe gesture recognizer model missing"
    exit 1
fi

print_success "🎉 All installation tests passed!"

# Build the workspace
print_status "Building ROS2 workspace..."
source /opt/ros/humble/setup.bash
colcon build --symlink-install

print_success "=================================================="
print_success "🎉 Robot Dog Petting Zoo System installation completed!"
print_success "=================================================="

echo ""
echo "📋 Next steps:"
echo "1. Source the environment: source setup_petting_zoo.sh"
echo "2. Start development!"
echo "3. Launch test camera: ros2 launch test_camera simple_camera_test.launch.py"
echo ""
echo "📚 Documentation:"
echo "- Check the README files in each package"
echo "- Review the implementation plan for Phase 1"
echo "- Test with your Go2 robot"
echo ""
echo "🐛 Troubleshooting:"
echo "- If you encounter issues, check the logs in the logs/ directory"
echo "- Ensure your robot is properly configured with go2_robot_sdk"
echo "- Verify WebRTC connection is working"
echo ""

print_success "Installation script completed successfully!" 