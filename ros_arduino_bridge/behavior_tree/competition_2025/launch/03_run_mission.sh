#!/bin/bash

# ╔════════════════════════════════════════════════════════════════╗
# ║       🌳 LAPTOP - COMPETITION BEHAVIOR TREE MISSION 🌳         ║
# ║              (Run this on Laptop AFTER nav ready)             ║
# ╚════════════════════════════════════════════════════════════════╝
#
# This is the MAIN COMPETITION SCRIPT!
# Runs the autonomous behavior tree mission.
#
# Prerequisites (ALL must be running):
# 1. Pi hardware (01_pi_hardware.sh on Raspberry Pi)
# 2. Laptop processing (02_laptop_processing.sh on Laptop)
# 3. Initial pose set in RViz (2D Pose Estimate tool)
#
# The behavior tree will execute the full autonomous mission:
# 1. Disease detection
# 2. Cargo loading (read color)
# 3. Maze navigation
# 4. Cargo delivery

echo "════════════════════════════════════════════════════════════════"
echo "   🌳 AUTONOMOUS COMPETITION MISSION - BEHAVIOR TREE 🌳"
echo "════════════════════════════════════════════════════════════════"
echo ""

# Source workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Change to behavior tree directory
cd "$(dirname "$0")/.."

echo "📋 Pre-Flight Checklist:"
echo ""

# Check 1: Nav2 running?
echo "🔍 [1/5] Checking Nav2 navigation stack..."
if ros2 node list 2>/dev/null | grep -q "controller_server\|bt_navigator"; then
    echo "       ✅ Nav2 detected"
else
    echo "       ❌ Nav2 NOT running!"
    echo ""
    echo "   Start laptop processing first:"
    echo "   → ./02_laptop_processing.sh"
    echo ""
    exit 1
fi

# Check 2: AMCL localization?
echo "🔍 [2/5] Checking AMCL localization..."
if ros2 topic list 2>/dev/null | grep -q "amcl_pose"; then
    echo "       ✅ AMCL detected"
else
    echo "       ❌ AMCL NOT running!"
    exit 1
fi

# Check 3: Pi hardware?
echo "🔍 [3/5] Checking Pi hardware..."
if ros2 topic list 2>/dev/null | grep -q "odom"; then
    echo "       ✅ Pi hardware detected"
else
    echo "       ⚠️  Warning: No odometry (robot won't move!)"
fi

# Check 4: Camera feed?
echo "🔍 [4/5] Checking camera feed..."
if ros2 topic list 2>/dev/null | grep -q "camera/image_raw"; then
    echo "       ✅ Camera detected"
else
    echo "       ⚠️  Warning: No camera (color detection will fail!)"
fi

# Check 5: py_trees installed?
echo "🔍 [5/5] Checking py_trees installation..."
if python3 -c "import py_trees, py_trees_ros" 2>/dev/null; then
    echo "       ✅ py_trees installed"
else
    echo "       ❌ py_trees NOT installed!"
    echo ""
    echo "   Install with:"
    echo "   pip3 install py_trees py_trees_ros"
    echo ""
    exit 1
fi

echo ""
echo "════════════════════════════════════════════════════════════════"
echo ""
echo "✅ All systems ready!"
echo ""
echo "🎯 Mission Overview:"
echo "════════════════════════════════════════════════════════════════"
echo ""
echo "   Phase 1: Disease Detection (optional)"
echo "      → Navigate to plant display"
echo "      → Run ML inference"
echo "      → Return to start"
echo ""
echo "   Phase 2: Cargo Loading"
echo "      → Navigate to loading bay"
echo "      → Reverse into bay"
echo "      → Read RGB sensor color"
echo "      → Exit bay"
echo ""
echo "   Phase 3: Maze Navigation"
echo "      → Navigate to delivery zone based on cargo color"
echo "      → Monitor camera for zone confirmation"
echo "      → Dynamic obstacle avoidance"
echo ""
echo "   Phase 4: Cargo Delivery"
echo "      → Verify correct zone"
echo "      → Reverse into delivery bay"
echo "      → Activate conveyor belt"
echo "      → If fails, add tipper servo tilt"
echo "      → Mission complete!"
echo ""
echo "════════════════════════════════════════════════════════════════"
echo ""
echo "⚠️  CRITICAL: Set robot initial pose in RViz NOW!"
echo ""
echo "   In RViz:"
echo "   1. Click '2D Pose Estimate' button (top toolbar)"
echo "   2. Click on map where robot is"
echo "   3. Drag to set orientation"
echo "   4. Green particles should converge around robot"
echo ""
read -p "Initial pose set? Press ENTER to start mission (Ctrl+C to cancel)..."
echo ""

echo "════════════════════════════════════════════════════════════════"
echo "   🚀 LAUNCHING AUTONOMOUS MISSION IN 3 SECONDS..."
echo "════════════════════════════════════════════════════════════════"
sleep 1
echo "   2..."
sleep 1
echo "   1..."
sleep 1
echo ""

echo "🌳 Starting Behavior Tree..."
echo ""

# Run the competition mission
python3 competition_mission.py

echo ""
echo "════════════════════════════════════════════════════════════════"
echo "   🏁 MISSION ENDED"
echo "════════════════════════════════════════════════════════════════"
echo ""
echo "Check terminal output above for mission status."
echo "Review RViz for robot final position."
echo ""
