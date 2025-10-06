#!/bin/bash

################################################################################
# RPI Camera Package - Full Processing Launch Script (Laptop)
# 
# This script launches BOTH color detection and disease detection nodes
# simultaneously on the laptop. Both nodes process the same camera feed
# in parallel for comprehensive image analysis.
#
# Usage: ./run_full_processing.sh
#
# Author: Allen Kizito Wachio
# Date: October 5, 2025
################################################################################

# Color codes for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
MAGENTA='\033[0;35m'
NC='\033[0m' # No Color

# Configuration
WORKSPACE_PATH="$HOME/ros2_ws"
USE_COMPRESSED=true
DISPLAY_OUTPUT=true

################################################################################
# Helper Functions
################################################################################

print_header() {
    echo -e "${MAGENTA}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║     RPI Camera Package - Full Processing Launcher (Laptop)    ║"
    echo "║              (Color Detection + Disease Detection)            ║"
    echo "╚════════════════════════════════════════════════════════════════╝"
    echo -e "${NC}"
}

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_error() {
    echo -e "${RED}❌ ERROR: $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  WARNING: $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ️  $1${NC}"
}

################################################################################
# Pre-flight Checks
################################################################################

print_header

echo -e "${CYAN}═══ Pre-flight Checks ═══${NC}\n"

# Check 1: Workspace exists
print_info "Checking workspace..."
if [ ! -d "$WORKSPACE_PATH" ]; then
    print_error "Workspace not found at $WORKSPACE_PATH"
    echo "Please ensure ROS2 workspace is built at $WORKSPACE_PATH"
    exit 1
fi
print_success "Workspace found"

# Check 2: ROS2 environment
print_info "Checking ROS2 installation..."
if ! command -v ros2 &> /dev/null; then
    print_error "ROS2 not found in PATH"
    echo "Please install ROS2 Humble or source your ROS2 installation"
    exit 1
fi
print_success "ROS2 found"

# Check 3: PyTorch (for disease detection)
print_info "Checking PyTorch installation..."
if ! python3 -c "import torch" 2>/dev/null; then
    print_error "PyTorch not found (required for disease detection)"
    echo "Install with: pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu"
    exit 1
fi
TORCH_VERSION=$(python3 -c "import torch; print(torch.__version__)" 2>/dev/null)
print_success "PyTorch installed (version: $TORCH_VERSION)"

# Check 4: OpenCV (for both)
print_info "Checking OpenCV installation..."
if ! python3 -c "import cv2" 2>/dev/null; then
    print_error "OpenCV not found (required for color detection)"
    echo "Install with: pip3 install opencv-python"
    exit 1
fi
print_success "OpenCV installed"

# Check 5: PIL (for disease detection)
print_info "Checking PIL installation..."
if ! python3 -c "import PIL" 2>/dev/null; then
    print_error "PIL (Pillow) not found (required for disease detection)"
    echo "Install with: pip3 install pillow"
    exit 1
fi
print_success "PIL installed"

# Check 6: Network configuration
print_info "Checking network configuration..."
if [ -z "$ROS_DOMAIN_ID" ]; then
    print_warning "ROS_DOMAIN_ID not set (should be 42)"
    echo "Add to ~/.bashrc: export ROS_DOMAIN_ID=42"
    echo ""
    echo "Setting temporarily for this session..."
    export ROS_DOMAIN_ID=42
else
    print_success "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
fi

if [ "$ROS_LOCALHOST_ONLY" != "0" ]; then
    print_warning "ROS_LOCALHOST_ONLY should be 0 for network communication"
    echo "Add to ~/.bashrc: export ROS_LOCALHOST_ONLY=0"
    echo ""
    echo "Setting temporarily for this session..."
    export ROS_LOCALHOST_ONLY=0
else
    print_success "ROS_LOCALHOST_ONLY=0"
fi

################################################################################
# Source Workspace
################################################################################

echo ""
echo -e "${CYAN}═══ Sourcing Workspace ═══${NC}\n"

print_info "Sourcing $WORKSPACE_PATH/install/setup.bash..."
if [ -f "$WORKSPACE_PATH/install/setup.bash" ]; then
    source "$WORKSPACE_PATH/install/setup.bash"
    print_success "Workspace sourced"
else
    print_error "Setup file not found. Please build the workspace first:"
    echo "  cd $WORKSPACE_PATH"
    echo "  colcon build --packages-select rpi_camera_package"
    exit 1
fi

################################################################################
# Check ML Model
################################################################################

echo ""
echo -e "${CYAN}═══ Checking ML Model ═══${NC}\n"

print_info "Looking for PyTorch model..."

MODEL_PATH="$WORKSPACE_PATH/install/rpi_camera_package/share/rpi_camera_package/models/model_ft_gpu.pth"

if [ -f "$MODEL_PATH" ]; then
    MODEL_SIZE=$(du -h "$MODEL_PATH" | cut -f1)
    print_success "Model found (size: $MODEL_SIZE)"
else
    print_error "Model not found at expected location"
    echo "Expected: $MODEL_PATH"
    echo ""
    echo "Please rebuild the package:"
    echo "  cd $WORKSPACE_PATH"
    echo "  colcon build --packages-select rpi_camera_package"
    exit 1
fi

################################################################################
# Check Camera Feed
################################################################################

echo ""
echo -e "${CYAN}═══ Checking Camera Feed ═══${NC}\n"

print_info "Looking for camera topics..."

# Wait a bit for discovery
sleep 2

if ros2 topic list 2>/dev/null | grep -q "/camera/image_raw"; then
    print_success "Camera topics found"
    
    # Check if compressed topic exists
    if ros2 topic list 2>/dev/null | grep -q "/camera/image_raw/compressed"; then
        print_success "Compressed image topic available"
    else
        print_warning "Compressed topic not found, will use raw images"
        USE_COMPRESSED=false
    fi
else
    print_warning "Camera topics not found"
    echo ""
    echo "Make sure the camera is running on the Raspberry Pi:"
    echo "  ./run_camera.sh"
    echo ""
    echo "Continuing anyway (nodes will wait for camera)..."
fi

################################################################################
# System Resources Check
################################################################################

echo ""
echo -e "${CYAN}═══ System Resources ═══${NC}\n"

# Check CPU cores
CPU_CORES=$(nproc)
print_info "CPU Cores: $CPU_CORES"

# Check available memory
AVAILABLE_MEM=$(free -h | grep "Mem:" | awk '{print $7}')
print_info "Available Memory: $AVAILABLE_MEM"

if [ $CPU_CORES -lt 4 ]; then
    print_warning "Low CPU count (< 4 cores). Performance may be affected."
fi

echo ""
print_warning "Running both detection systems will use 60-80% CPU"
print_info "Each node will open its own display window"

################################################################################
# Launch Configuration
################################################################################

echo ""
echo -e "${CYAN}═══ Launch Configuration ═══${NC}\n"

print_info "Use Compressed Images: $USE_COMPRESSED"
print_info "Display Output: $DISPLAY_OUTPUT (2 windows)"
print_info "Launch File: full_processing.launch.py"
print_info "Namespace: laptop"

echo ""
print_info "Color Detection: Red, Green, Blue, Yellow objects"
print_info "Disease Detection: Healthy, Early Blight, Late Blight"

################################################################################
# Launch Full Processing
################################################################################

echo ""
echo -e "${CYAN}═══ Starting Full Processing ═══${NC}\n"

print_success "Launching both color and disease detection..."
echo ""
print_info "📊 Topics published:"
print_info "  - /color_detection/processed_image"
print_info "  - /disease_detection/result"
print_info "  - /disease_detection/annotated_image"
echo ""
print_info "🖼️  Two display windows will open:"
print_info "  - Color Detection window"
print_info "  - Disease Detection window"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop both nodes${NC}"
echo ""
echo "────────────────────────────────────────────────────────────────"
echo ""

# Launch with all parameters
ros2 launch rpi_camera_package full_processing.launch.py \
    use_compressed:=$USE_COMPRESSED \
    display:=$DISPLAY_OUTPUT

# Capture exit code
EXIT_CODE=$?

echo ""
echo "────────────────────────────────────────────────────────────────"
echo ""

if [ $EXIT_CODE -eq 0 ] || [ $EXIT_CODE -eq 130 ]; then
    # Exit code 130 is Ctrl+C (SIGINT)
    print_success "Full processing stopped successfully"
else
    print_error "Full processing exited with error code $EXIT_CODE"
    exit $EXIT_CODE
fi
