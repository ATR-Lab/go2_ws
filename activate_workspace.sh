#!/bin/bash
# activate_workspace.sh - Robot Dog Petting Zoo workspace activation
#
# DESCRIPTION:
#   This script sets up the complete development environment for the Robot Dog Petting Zoo.
#   It activates the virtual environment, configures PYTHONPATH for ROS2 compatibility,
#   and sources all necessary ROS2 environments.
#
# USAGE:
#   source activate_workspace.sh
#
# REQUIREMENTS:
#   - Virtual environment must exist (run install_petting_zoo_dependencies.sh first)
#   - ROS2 Humble installed
#   - Workspace must be built (colcon build)
#
# WHAT IT DOES:
#   1. Activates the petting_zoo_venv virtual environment
#   2. Configures PYTHONPATH to include virtual environment packages
#   3. Sources ROS2 Humble environment
#   4. Sources the workspace overlay
#   5. Verifies the setup is working correctly

# Check if script is being sourced
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "Error: This script must be sourced, not executed."
    echo "Usage: source $(basename ${BASH_SOURCE[0]})"
    exit 1
fi

# Find the workspace root (where this script is located)
WORKSPACE_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
VENV_PATH="$WORKSPACE_ROOT/petting_zoo_venv"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if virtual environment exists
check_venv() {
    if [ ! -d "$VENV_PATH" ]; then
        log_error "Virtual environment not found at $VENV_PATH"
        echo ""
        echo "Please run the setup script first:"
        echo "  ./install_petting_zoo_dependencies.sh"
        echo ""
        return 1
    fi
    
    if [ ! -f "$VENV_PATH/bin/activate" ]; then
        log_error "Virtual environment activation script not found"
        return 1
    fi
}

# Activate virtual environment and configure PYTHONPATH
activate_venv() {
    log_info "[1/4] Activating virtual environment..."
    source "$VENV_PATH/bin/activate"
    
    log_info "[2/4] Configuring PYTHONPATH for ROS2 compatibility..."
    # Get the virtual environment's site-packages directory
    VENV_SITE_PACKAGES=$(python -c "import site; print(site.getsitepackages()[0])")
    
    # Add virtual environment packages to PYTHONPATH so ROS2 nodes can find them
    export PYTHONPATH="$VENV_SITE_PACKAGES:$PYTHONPATH"
    
    # Note: PyQt5 is removed from virtual environment during installation
    # to prevent Qt version conflicts with ROS2's system Qt bindings
    
    log_success "Virtual environment activated with PYTHONPATH configured"
    log_success "Qt conflicts prevented (PyQt5 removed from venv during install)"
}

# Source ROS2 environment
source_ros2() {
    log_info "[3/4] Sourcing ROS2 Humble environment..."
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source "/opt/ros/humble/setup.bash"
        log_success "ROS2 Humble environment sourced"
    else
        log_error "ROS2 Humble not found at /opt/ros/humble"
        return 1
    fi
}

# Source workspace overlay
source_workspace() {
    log_info "[4/4] Sourcing workspace overlay..."
    if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
        cd "$WORKSPACE_ROOT"
        source install/setup.bash
        log_success "Workspace overlay sourced"
    else
        log_warning "Workspace not built yet. Run 'colcon build' in $WORKSPACE_ROOT"
        log_warning "Some packages may not be available until workspace is built"
    fi
}

# Verify setup
verify_setup() {
    log_info "Verifying environment setup..."
    
    # Check ROS2 command availability
    if ! command -v ros2 &> /dev/null; then
        log_error "ROS2 command not available"
        return 1
    fi
    
    # Check Python environment
    local python_path=$(which python)
    log_info "Python: $python_path"
    
    # Test critical imports
    if python -c "from pyzbar import pyzbar; print('pyzbar available')" 2>/dev/null; then
        log_success "pyzbar is available (QR detection enabled)"
    else
        log_warning "pyzbar not available (QR detection will be disabled)"
    fi
    
    if python -c "from boxmot import ByteTrack; print('boxmot available')" 2>/dev/null; then
        log_success "boxmot is available (human tracking enabled)"
    else
        log_warning "boxmot not available (human tracking will be disabled)"
    fi
    
    # Check NumPy version compatibility with ROS2 Humble
    if python -c "import numpy as np; assert np.__version__.startswith('1.'), f'NumPy {np.__version__} incompatible with ROS2 Humble'; print(f'NumPy {np.__version__} compatible')" 2>/dev/null; then
        log_success "NumPy version is compatible with ROS2 Humble"
    else
        log_error "NumPy version incompatible with ROS2 Humble cv_bridge"
        log_error "Run: ./install_petting_zoo_dependencies.sh --fix-deps"
        return 1
    fi
    
    # Check OpenCV version compatibility
    if python -c "import cv2; print(f'OpenCV {cv2.__version__} available')" 2>/dev/null; then
        local opencv_version=$(python -c "import cv2; print(cv2.__version__)" 2>/dev/null)
        if [[ $opencv_version == 4.8.* ]]; then
            log_success "OpenCV version $opencv_version is compatible"
        else
            log_warning "OpenCV version $opencv_version may cause compatibility issues"
            log_warning "Recommended version: 4.8.x. Run --fix-deps to update."
        fi
    else
        log_error "OpenCV not available"
        return 1
    fi
    
    # Verify PyQt5 is NOT in virtual environment (this prevents Qt conflicts)
    if python -c "import PyQt5" 2>/dev/null; then
        log_warning "PyQt5 found in virtual environment - this may cause Qt conflicts"
        log_warning "Run: ./install_petting_zoo_dependencies.sh --fix-deps"
    else
        log_success "PyQt5 correctly absent from venv (prevents Qt conflicts)"
    fi
    
    # Check if workspace packages are available
    if ros2 pkg list | grep -q "go2_robot_ui"; then
        log_success "go2_robot_ui package found"
    else
        log_warning "go2_robot_ui package not found. Workspace may need to be built."
    fi
    
    return 0
}

# Environment variables are configured dynamically - no .env file needed

# Main execution
main() {
    echo "======================================"
    echo "🤖 Robot Dog Petting Zoo Activation"
    echo "======================================"
    echo ""
    
    log_info "Workspace root: $WORKSPACE_ROOT"
    echo ""
    
    # Check virtual environment exists
    if ! check_venv; then
        return 1
    fi
    
    # Activate virtual environment with PYTHONPATH configuration
    if ! activate_venv; then
        return 1
    fi
    
    # Source ROS2 environment
    if ! source_ros2; then
        return 1
    fi
    
    # Source workspace overlay
    if ! source_workspace; then
        return 1
    fi
    
    echo ""
    
    # Verify setup
    if verify_setup; then
        log_success "Environment verification passed!"
    else
        log_warning "Environment verification had issues, but activation completed"
    fi
    
    echo ""
    log_success "🎉 Petting Zoo workspace activation complete!"
    echo ""
    echo "Environment Details:"
    echo "  Workspace: $WORKSPACE_ROOT"
    echo "  Python: $(which python)"
    echo "  Virtual Environment: $VENV_PATH"
    echo "  PYTHONPATH configured: Yes"
    echo ""
    echo "📦 Available packages:"
    echo "   - human_interaction (AI behavior and gesture recognition)"
    echo "   - go2_robot_ui (Robot control interface with QR detection)"
    echo "   - go2_robot_sdk (Robot communication and control)"
    echo "   - speech_processor (Text-to-speech functionality)"
    echo "   - navigation_manager (Autonomous navigation)"
    echo "   - test_camera (Camera testing and development)"
    echo ""
    echo "🚀 Ready for development! You can now:"
    echo "   - Build workspace: colcon build"
    echo "   - Run UI with QR detection: ros2 run go2_robot_ui go2_robot_ui"
    echo "   - Test camera: ros2 launch test_camera simple_camera_test.launch.py"
    echo "   - Run human interaction: ros2 run human_interaction human_detection_node"
    echo ""
}

# Execute main function
main
