#!/bin/bash

################################################################################
# RPI Camera Package - Disease Detection Launch Script (Laptop)
# 
# This script launches the disease detection node on the laptop with all
# necessary parameters pre-configured. It uses PyTorch ML model to classify
# potato leaf diseases (Healthy, Early Blight, Late Blight).
#
# Usage: ./run_disease_detection.sh
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
NC='\033[0m' # No Color

# Configuration
WORKSPACE_PATH="$HOME/ros2_ws"
USE_COMPRESSED=true
DISPLAY_OUTPUT=true
INFERENCE_RATE=1.0
CONFIDENCE_THRESHOLD=0.0
PUBLISH_ANNOTATED=true

################################################################################
# Helper Functions
################################################################################

print_header() {
    echo -e "${CYAN}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║    RPI Camera Package - Disease Detection Launcher (Laptop)   ║"
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

# Check 3: PyTorch
print_info "Checking PyTorch installation..."
if ! python3 -c "import torch" 2>/dev/null; then
    print_error "PyTorch not found"
    echo "Install with: pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cpu"
    exit 1
fi
TORCH_VERSION=$(python3 -c "import torch; print(torch.__version__)" 2>/dev/null)
print_success "PyTorch installed (version: $TORCH_VERSION)"

# Check 4: PIL (Pillow)
print_info "Checking PIL installation..."
if ! python3 -c "import PIL" 2>/dev/null; then
    print_error "PIL (Pillow) not found"
    echo "Install with: pip3 install pillow"
    exit 1
fi
print_success "PIL installed"

# Check 5: OpenCV (for display)
print_info "Checking OpenCV installation..."
if ! python3 -c "import cv2" 2>/dev/null; then
    print_warning "OpenCV not found (needed for display)"
    echo "Install with: pip3 install opencv-python"
    echo ""
    echo "Continuing anyway (display may not work)..."
else
    print_success "OpenCV installed"
fi

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
    echo "Continuing anyway (node will wait for camera)..."
fi

################################################################################
# Launch Configuration
################################################################################

echo ""
echo -e "${CYAN}═══ Launch Configuration ═══${NC}\n"

print_info "Use Compressed Images: $USE_COMPRESSED"
print_info "Display Output: $DISPLAY_OUTPUT"
print_info "Inference Rate: $INFERENCE_RATE Hz"
print_info "Confidence Threshold: $CONFIDENCE_THRESHOLD"
print_info "Publish Annotated Images: $PUBLISH_ANNOTATED"
print_info "Model: ResNet18 (3 classes)"
print_info "Launch File: disease_detection.launch.py"
print_info "Namespace: laptop"

################################################################################
# Launch Disease Detection Node
################################################################################

echo ""
echo -e "${CYAN}═══ Starting Disease Detection Node ═══${NC}\n"

print_success "Launching disease detection with ML model..."
echo ""
print_info "Disease classes: 🟢 Healthy  🟠 Early Blight  🔴 Late Blight"
print_info "Results published to: /disease_detection/result"
print_info "Annotated images: /disease_detection/annotated_image"
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop${NC}"
echo ""
echo "────────────────────────────────────────────────────────────────"
echo ""

# Launch with all parameters
ros2 launch rpi_camera_package disease_detection.launch.py \
    use_compressed:=$USE_COMPRESSED \
    display:=$DISPLAY_OUTPUT \
    inference_rate:=$INFERENCE_RATE \
    confidence_threshold:=$CONFIDENCE_THRESHOLD \
    publish_annotated:=$PUBLISH_ANNOTATED

# Capture exit code
EXIT_CODE=$?

echo ""
echo "────────────────────────────────────────────────────────────────"
echo ""

if [ $EXIT_CODE -eq 0 ] || [ $EXIT_CODE -eq 130 ]; then
    # Exit code 130 is Ctrl+C (SIGINT)
    print_success "Disease detection node stopped successfully"
else
    print_error "Disease detection node exited with error code $EXIT_CODE"
    exit $EXIT_CODE
fi
