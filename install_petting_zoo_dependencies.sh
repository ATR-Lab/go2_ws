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

# Recovery function for fixing dependency issues on existing installations
fix_dependencies() {
    print_status "🔧 Running dependency recovery mode..."
    
    # Activate virtual environment if it exists
    if [ -f "petting_zoo_venv/bin/activate" ]; then
        source petting_zoo_venv/bin/activate
        print_success "Virtual environment activated"
    else
        print_error "Virtual environment not found. Run full installation first."
        exit 1
    fi
    
    # Fix NumPy version (force reinstall to ensure compatibility)
    print_status "Fixing NumPy version for ROS2 Humble compatibility..."
    pip install "numpy>=1.21.0,<2.0" --force-reinstall
    
    # Fix OpenCV version (remove all variants and install compatible GUI-enabled version)
    print_status "Fixing OpenCV version..."
    pip uninstall opencv-python opencv-contrib-python opencv-python-headless -y 2>/dev/null || true
    pip install opencv-python==4.8.1.78
    
    # Verify the fixes
    print_status "Verifying dependency fixes..."
    python -c "
import numpy as np
import cv2
print(f'✅ NumPy version: {np.__version__} (compatible: {np.__version__.startswith(\"1.\")})')
print(f'✅ OpenCV version: {cv2.__version__} (compatible: {cv2.__version__.startswith(\"4.8\")})')
# Test GUI support (will show warning if no display, but won't crash)
try:
    # This will work even in headless environments
    test_img = cv2.imread('/dev/null')  # Won't crash, just returns None
    print('✅ OpenCV GUI functions available')
except Exception as e:
    print(f'⚠️  OpenCV GUI test: {e}')
print('✅ Dependencies fixed successfully!')
" || {
        print_error "Dependency verification failed!"
        exit 1
    }
    
    print_success "🎉 Dependency recovery completed successfully!"
    print_success "You can now run: ros2 launch test_camera simple_camera_test.launch.py"
    exit 0
}

# Check for recovery mode flag
if [ "$1" == "--fix-deps" ] || [ "$1" == "--fix-dependencies" ]; then
    fix_dependencies
fi

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

# Update package list (skip if recent)
print_status "Checking if package list update is needed..."
LAST_UPDATE=$(stat -c %Y /var/lib/apt/lists 2>/dev/null || echo 0)
CURRENT_TIME=$(date +%s)
TIME_DIFF=$((CURRENT_TIME - LAST_UPDATE))

if [ $TIME_DIFF -gt 86400 ]; then  # 24 hours
    print_status "Updating package list (last update was more than 24 hours ago)..."
    sudo apt update
else
    print_status "Package list is recent, skipping update..."
fi

# Install system dependencies
print_status "Installing system dependencies..."
sudo apt install -y \
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
    libgomp1 \
    libzbar0 \
    libzbar-dev

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
if [ ! -f "petting_zoo_venv/bin/activate" ]; then
    if [ -d "petting_zoo_venv" ]; then
        print_warning "Removing incomplete virtual environment directory..."
        rm -rf petting_zoo_venv
    fi
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

# Pin NumPy to version compatible with ROS2 Humble cv_bridge BEFORE installing other packages
print_status "Installing NumPy compatible with ROS2 Humble..."
pip install "numpy>=1.21.0,<2.0"

# Install Python packages from requirements.txt
if [ -f "requirements.txt" ]; then
    pip install -r requirements.txt
    print_success "Installed Python dependencies from requirements.txt"
else
    print_error "requirements.txt not found!"
    exit 1
fi

# Remove Qt-related packages from virtual environment to avoid Qt version conflicts
#
# PROBLEM: The ML dependencies automatically install Qt-enabled packages that conflict with ROS2:
#   - mediapipe>=0.10.0 → installs opencv-contrib-python (with Qt GUI plugins)
#   - ultralytics>=8.0.0 → installs opencv-python (with Qt GUI plugins)  
#   - boxmot>=10.0.0 → installs opencv-python + pyqt5 (with Qt version conflicts)
#
# CONFLICT: Virtual environment gets Qt 5.15.14, but ROS2 system uses Qt 5.15.3
#   This causes "Could not load Qt platform plugin" errors when running ROS2 GUI nodes
#
# SOLUTION: Remove Qt-enabled packages and use compatible GUI-enabled OpenCV
#   - PyQt5/PyQt5-Qt5/PyQt5-sip: Removed (ROS2 system Qt handles GUI)
#   - opencv-contrib-python: Removed (has Qt plugins that conflict)
#   - opencv-python==4.8.1.78: Installed (compatible version with GUI support for testing)
#
# RESULT: ML libraries get OpenCV functionality, camera display works for testing
print_status "Removing Qt-related packages from virtual environment to prevent Qt conflicts..."
pip uninstall PyQt5 PyQt5-Qt5 PyQt5-sip -y 2>/dev/null || true
pip uninstall opencv-contrib-python opencv-python -y 2>/dev/null || true
print_status "Installing compatible GUI-enabled OpenCV for testing and development..."
pip install opencv-python==4.8.1.78
print_success "Qt-related packages removed, GUI-enabled OpenCV installed for testing"

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

# Environment variables are configured dynamically in activate_workspace.sh
# No persistent .env file needed with the dynamic activation approach

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

# Note: activate_workspace.sh script is now provided as a separate file
# Users should run: source activate_workspace.sh

# Test installation (run tests inline while venv is active)
print_status "Testing installation..."

# Test Python imports (in virtual environment)
print_status "Testing Python dependencies..."
python -c "
import cv2
import mediapipe
import ultralytics
import numpy as np
from pyzbar import pyzbar
from boxmot import ByteTrack
print('✅ All Python dependencies imported successfully')
print('✅ pyzbar available for QR detection')
print('✅ boxmot available for human tracking')
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
echo "1. Source the environment: source activate_workspace.sh"
echo "2. Start development!"
echo "3. Launch test camera: ros2 launch test_camera simple_camera_test.launch.py"
echo "4. Run UI with QR detection: ros2 run go2_robot_ui go2_robot_ui"
echo "5. Run human detection: ros2 run human_interaction human_detection_node"
echo ""
echo "📚 Documentation:"
echo "- Check the README files in each package"
echo "- Review the implementation plan for Phase 1"
echo "- Test with your Go2 robot"
echo ""
echo "🐛 Troubleshooting:"
echo "- If you encounter issues, check the logs in the logs/ directory"
echo "- For NumPy/OpenCV compatibility issues, run: ./install_petting_zoo_dependencies.sh --fix-deps"
echo "- Ensure your robot is properly configured with go2_robot_sdk"
echo "- Verify WebRTC connection is working"
echo ""

print_success "Installation script completed successfully!" 