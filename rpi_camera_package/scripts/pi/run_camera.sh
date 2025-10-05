#!/bin/bash

################################################################################
# RPI Camera Package - Camera Launch Script (Raspberry Pi)
# 
# This script launches the camera node on Raspberry Pi with all necessary
# parameters pre-configured. Just run this script and the camera will start
# publishing compressed images over the network.
#
# Usage: ./run_camera.sh
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
WIDTH=1280
HEIGHT=720
FRAMERATE=15
JPEG_QUALITY=80

################################################################################
# Helper Functions
################################################################################

print_header() {
    echo -e "${CYAN}"
    echo "╔════════════════════════════════════════════════════════════════╗"
    echo "║          RPI Camera Package - Camera Launcher (Pi)            ║"
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

# Check 2: rpicam-apps installed
print_info "Checking rpicam-apps installation..."
if ! command -v rpicam-vid &> /dev/null; then
    print_error "rpicam-vid not found"
    echo ""
    echo "rpicam-apps is required for Raspberry Pi Camera Module 2"
    echo "Install with: sudo apt-get install rpicam-apps"
    echo ""
    exit 1
fi
print_success "rpicam-vid found"

# Check 3: Camera detection
print_info "Checking camera detection..."
if command -v vcgencmd &> /dev/null; then
    CAMERA_STATUS=$(vcgencmd get_camera 2>&1)
    if echo "$CAMERA_STATUS" | grep -q "detected=1"; then
        print_success "Camera detected by vcgencmd"
    else
        print_warning "Camera not detected by vcgencmd"
        echo "Status: $CAMERA_STATUS"
        echo ""
        echo "Trying to continue anyway..."
    fi
else
    print_warning "vcgencmd not available (skip camera detection)"
fi

# Check 4: ROS2 environment
print_info "Checking ROS2 installation..."
if ! command -v ros2 &> /dev/null; then
    print_error "ROS2 not found in PATH"
    echo "Please install ROS2 Humble or source your ROS2 installation"
    exit 1
fi
print_success "ROS2 found"

# Check 5: Network configuration
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
# Launch Configuration
################################################################################

echo ""
echo -e "${CYAN}═══ Launch Configuration ═══${NC}\n"

print_info "Resolution: ${WIDTH}x${HEIGHT}"
print_info "Frame Rate: ${FRAMERATE} FPS"
print_info "JPEG Quality: $JPEG_QUALITY%"
print_info "Launch File: camera_rpicam.launch.py"
print_info "Camera Driver: rpicam-vid (libcamera)"

################################################################################
# Launch Camera Node
################################################################################

echo ""
echo -e "${CYAN}═══ Starting Camera Node ═══${NC}\n"

print_success "Launching camera with rpicam-vid (libcamera)..."
echo ""
echo -e "${YELLOW}Press Ctrl+C to stop${NC}"
echo ""
echo "────────────────────────────────────────────────────────────────"
echo ""

# Launch with all parameters
ros2 launch rpi_camera_package camera_rpicam.launch.py \
    width:=$WIDTH \
    height:=$HEIGHT \
    framerate:=$FRAMERATE \
    jpeg_quality:=$JPEG_QUALITY

# Capture exit code
EXIT_CODE=$?

echo ""
echo "────────────────────────────────────────────────────────────────"
echo ""

if [ $EXIT_CODE -eq 0 ] || [ $EXIT_CODE -eq 130 ]; then
    # Exit code 130 is Ctrl+C (SIGINT)
    print_success "Camera node stopped successfully"
else
    print_error "Camera node exited with error code $EXIT_CODE"
    exit $EXIT_CODE
fi
