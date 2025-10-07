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

echo "🚀 Starting laptop processing..."
echo ""

# Launch laptop Nav2 using YOUR EXISTING launch file!
# This uses: ros_arduino_bridge/deployment/laptop/launch/laptop_navigation.launch.py
ros2 launch ros_arduino_bridge laptop_navigation.launch.py \
    map_file:="${MAP_NAME}.yaml" \
    map_path:="$MAP_PATH/" \
    use_rviz:=true &

# Wait for Nav2 to start
sleep 5

echo ""
echo "🎥 Starting vision processing..."
echo ""

# Launch camera processing using YOUR EXISTING launch file!
# This uses: rpi_camera_package/launch/laptop/full_processing.launch.py
ros2 launch rpi_camera_package full_processing.launch.py \
    use_compressed:=true \
    display_color:=false \
    display_disease:=false \
    inference_rate:=1.0

echo ""
echo "════════════════════════════════════════════════════════════════"
echo "   🛑 Laptop processing stopped"
echo "════════════════════════════════════════════════════════════════"
