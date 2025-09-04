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
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libgomp1 \
    libgstreamer1.0-0 \
    libgstreamer-plugins-base1.0-0 \
    libgstreamer-plugins-bad1.0-0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-tools \
    gstreamer1.0-x \
    gstreamer1.0-alsa \
    gstreamer1.0-gl \
    gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 \
    gstreamer1.0-pulseaudio

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

# Create ROS2 packages
print_status "Creating ROS2 packages..."

# Create human_interaction package
if [ ! -d "src/human_interaction" ]; then
    cd src
    ros2 pkg create --build-type ament_python human_interaction --dependencies rclpy sensor_msgs geometry_msgs std_msgs go2_interfaces cv_bridge
    cd ..
    print_success "Created human_interaction package"
else
    print_warning "human_interaction package already exists"
fi

# Create petting_zoo_behavior package
if [ ! -d "src/petting_zoo_behavior" ]; then
    cd src
    ros2 pkg create --build-type ament_python petting_zoo_behavior --dependencies rclpy go2_interfaces std_msgs geometry_msgs nav_msgs
    cd ..
    print_success "Created petting_zoo_behavior package"
else
    print_warning "petting_zoo_behavior package already exists"
fi

# Create enhanced_collision_monitor package
if [ ! -d "src/enhanced_collision_monitor" ]; then
    cd src
    ros2 pkg create --build-type ament_python enhanced_collision_monitor --dependencies rclpy nav2_costmap_2d geometry_msgs sensor_msgs
    cd ..
    print_success "Created enhanced_collision_monitor package"
else
    print_warning "enhanced_collision_monitor package already exists"
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
echo "   - petting_zoo_behavior" 
echo "   - enhanced_collision_monitor"
echo ""
echo "🚀 To build: colcon build"
echo "🎯 To run: ros2 launch go2_robot_sdk robot.launch.py"
EOF

chmod +x setup_petting_zoo.sh
print_success "Created setup_petting_zoo.sh script"

# Create a test script
cat > test_installation.sh << 'EOF'
#!/bin/bash
# Test script to verify installation

echo "🧪 Testing Robot Dog Petting Zoo installation..."

# Test Python imports
python3 -c "
import cv2
import mediapipe
import ultralytics
import numpy as np
import rclpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
print('✅ All Python dependencies imported successfully')
"

# Test ROS2 packages
if [ -d "src/human_interaction" ] && [ -d "src/petting_zoo_behavior" ] && [ -d "src/enhanced_collision_monitor" ]; then
    echo "✅ All ROS2 packages created successfully"
else
    echo "❌ Some ROS2 packages missing"
    exit 1
fi

# Test YOLO model
if [ -f "models/yolov8n.pt" ]; then
    echo "✅ YOLOv8 model downloaded successfully"
else
    echo "❌ YOLOv8 model missing"
    exit 1
fi

echo "🎉 Installation test completed successfully!"
EOF

chmod +x test_installation.sh
print_success "Created test_installation.sh script"

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
echo "2. Test installation: ./test_installation.sh"
echo "3. Build workspace: colcon build"
echo "4. Start development!"
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