#!/bin/bash

# ╔════════════════════════════════════════════════════════════════╗
# ║         💻 LAPTOP - COMPETITION PROCESSING & NAV2 💻           ║
# ║                    (Run this on the Laptop)                   ║
# ╚════════════════════════════════════════════════════════════════╝
#
# This script runs on the LAPTOP and handles compute-intensive tasks:
# - Nav2 navigation stack (path planning, obstacle avoidance, AMCL)
# - Map server (loads competition field map)
# - ML disease detection model
# - Camera color detection processing
# - RViz visualization
#
# Prerequisites:
# - Pi must be running (01_pi_hardware.sh)
# - Competition map must exist: ~/ros2_ws/maps/gamefield.yaml

echo "════════════════════════════════════════════════════════════════"
echo "  💻 STARTING LAPTOP PROCESSING FOR COMPETITION 💻"
echo "════════════════════════════════════════════════════════════════"
echo ""

# Source workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Map configuration
MAP_NAME="gamefield"
MAP_PATH="$HOME/ros2_ws/maps"

# Check if map exists
if [ ! -f "$MAP_PATH/${MAP_NAME}.yaml" ]; then
    echo "❌ Competition map NOT found!"
    echo "   Expected: $MAP_PATH/${MAP_NAME}.yaml"
    echo ""
    echo "Available maps:"
    ls -1 "$MAP_PATH"/*.yaml 2>/dev/null | sed 's|.*/||' | sed 's|\.yaml||' || echo "  (none)"
    echo ""
    read -p "Enter map name to use (without .yaml): " MAP_NAME
    
    if [ ! -f "$MAP_PATH/${MAP_NAME}.yaml" ]; then
        echo "❌ Map '$MAP_NAME' not found. Exiting."
        exit 1
    fi
fi

echo "✅ Using map: $MAP_NAME"
echo "   Path: $MAP_PATH/${MAP_NAME}.yaml"
echo ""

# Check if Pi is running
echo "🔍 Checking if Pi hardware is running..."
if ros2 topic list 2>/dev/null | grep -q "odom"; then
    echo "   ✅ Pi hardware detected (odometry publishing)"
else
    echo "   ❌ Pi hardware NOT detected!"
    echo ""
    echo "   Please start Pi first:"
    echo "   On Raspberry Pi, run: ./01_pi_hardware.sh"
    echo ""
    read -p "Continue anyway? (y/N): " continue_choice
    if [[ ! "$continue_choice" =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

echo ""
echo "════════════════════════════════════════════════════════════════"
echo "   🧠 LAPTOP WILL RUN (subscribes to Pi topics):"
echo "════════════════════════════════════════════════════════════════"
echo "   Nav2 Navigation Stack:"
echo "     - Map Server (gamefield.pgm)"
echo "     - AMCL Localization (/amcl_pose)"
echo "     - Path Planner (global/local)"
echo "     - Controller (DWB path follower)"
echo "     - Recovery Behaviors"
echo ""
echo "   Vision Processing:"
echo "     - Disease Detection ML Model"
echo "     - Camera Color Detection (HSV)"
echo ""
echo "   Visualization:"
echo "     - RViz (map, robot, paths, obstacles)"
echo ""
echo "════════════════════════════════════════════════════════════════"
echo ""

echo "🚀 Starting laptop processing using YOUR EXISTING SCRIPTS..."
echo ""

# Get the absolute paths to your existing scripts
NAV_SCRIPT="$HOME/ros2_ws/src/ros_arduino_bridge/deployment/scripts/laptop/02c_slam_navigation_mode.sh"
VISION_SCRIPT="$HOME/ros2_ws/src/rpi_camera_package/scripts/laptop/run_full_processing.sh"

# Check if scripts exist
if [ ! -f "$NAV_SCRIPT" ]; then
    echo "❌ Navigation script not found: $NAV_SCRIPT"
    exit 1
fi

if [ ! -f "$VISION_SCRIPT" ]; then
    echo "❌ Vision processing script not found: $VISION_SCRIPT"
    exit 1
fi

# Launch Navigation Stack using your deployment script (runs in background)
echo "   🗺️  Starting Nav2 navigation stack..."
echo "      Using: deployment/scripts/laptop/02_navigation_mode.sh"
echo ""

# Run navigation script in background with map name
$NAV_SCRIPT "$MAP_NAME" &
NAV_PID=$!

echo "   ⏳ Waiting for Nav2 to initialize (10 seconds)..."
sleep 10

# Check if Nav2 is actually running
if ros2 node list 2>/dev/null | grep -q "amcl"; then
    echo "   ✅ Nav2 stack running"
else
    echo "   ⚠️  Nav2 might not be ready yet, continuing anyway..."
fi

echo ""
echo "🎥 Starting vision processing..."
echo "   Using: rpi_camera_package/scripts/laptop/run_full_processing.sh"
echo ""

# Launch vision processing (runs in foreground)
# This will keep the terminal alive and show processing output
$VISION_SCRIPT

# When vision script exits (Ctrl+C), clean up navigation
echo ""
echo "════════════════════════════════════════════════════════════════"
echo "   🛑 Stopping laptop processing..."
echo "════════════════════════════════════════════════════════════════"

# Kill navigation process
kill $NAV_PID 2>/dev/null

echo "   ✅ All laptop processing stopped"

echo ""
echo "════════════════════════════════════════════════════════════════"
echo "   🛑 Laptop processing stopped"
echo "════════════════════════════════════════════════════════════════"
