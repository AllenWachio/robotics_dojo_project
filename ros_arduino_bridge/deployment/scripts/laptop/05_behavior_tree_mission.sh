#!/bin/bash

# LAPTOP SCRIPT - Behavior Tree Autonomous Mission
# This script runs the py_trees behavior tree on the laptop for autonomous navigation
# 
# PREREQUISITES:
# - Pi must be running hardware nodes (Arduino + LiDAR)
# - Laptop must be running navigation mode (Option A, B, or C)
# - A map must already be loaded
# - Robot's initial pose must be set in RViz
#
# The behavior tree will:
# 1. Navigate to waypoint (2.1, 0.0)
# 2. Execute Task 1 (print "hello")
# 3. Navigate to waypoint (0, -1.2)
# 4. Execute Task 2 (print "hi")
#
# Usage: ./05_behavior_tree_mission.sh

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║      🤖 Behavior Tree Autonomous Mission Launcher 🌳          ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

# Source the workspace
source /opt/ros/humble/setup.bash
source $HOME/ros2_ws/install/setup.bash

echo "📋 Pre-flight Checklist:"
echo ""

# Check 1: Is navigation running?
echo "🔍 Checking if Nav2 navigation is running..."
if ros2 node list 2>/dev/null | grep -q "controller_server\|bt_navigator"; then
    echo "   ✅ Nav2 navigation stack detected"
else
    echo "   ❌ Nav2 navigation stack NOT running!"
    echo ""
    echo "   Please start navigation first using ONE of these options:"
    echo "   - ./02_navigation_mode.sh [map_name]      (AMCL navigation)"
    echo "   - ./02b_slam_localization_mode.sh [map]   (SLAM localization only)"
    echo "   - ./02c_slam_navigation_mode.sh [map]     (SLAM navigation - recommended)"
    echo ""
    exit 1
fi

# Check 2: Is AMCL or SLAM Toolbox providing localization?
echo "🔍 Checking localization system..."
if ros2 topic list 2>/dev/null | grep -q "amcl_pose"; then
    echo "   ✅ AMCL localization detected"
    LOCALIZATION="AMCL"
elif ros2 topic list 2>/dev/null | grep -q "slam_toolbox"; then
    echo "   ✅ SLAM Toolbox localization detected"
    LOCALIZATION="SLAM"
else
    echo "   ❌ No localization system detected!"
    echo "   The behavior tree needs /amcl_pose topic."
    echo "   Make sure navigation mode is fully started."
    exit 1
fi

# Check 3: Is the Pi hardware running?
echo "🔍 Checking if Pi hardware nodes are running..."
if ros2 topic list 2>/dev/null | grep -q "odom"; then
    echo "   ✅ Robot hardware detected (odometry publishing)"
else
    echo "   ⚠️  Warning: No odometry detected. Is the Pi running?"
    echo "   The robot may not move without hardware nodes."
fi

# Check 4: Is py_trees installed?
echo "🔍 Checking if py_trees is installed..."
if python3 -c "import py_trees" 2>/dev/null; then
    echo "   ✅ py_trees Python package found"
else
    echo "   ❌ py_trees NOT installed!"
    echo ""
    echo "   Install with: pip3 install py_trees py_trees_ros"
    echo ""
    exit 1
fi

echo ""
echo "════════════════════════════════════════════════════════════════"
echo ""
echo "🚀 All checks passed! Starting behavior tree mission..."
echo ""
echo "📍 Mission Waypoints:"
echo "   1. Navigate to (2.1, 0.0)"
echo "   2. Execute Task 1"
echo "   3. Navigate to (0, -1.2)"
echo "   4. Execute Task 2"
echo ""
echo "💡 IMPORTANT:"
echo "   - Make sure you've set the robot's initial pose in RViz!"
echo "   - Use '2D Pose Estimate' tool to set starting location"
echo "   - Watch RViz for navigation progress"
echo "   - Press Ctrl+C to stop the mission"
echo ""
echo "════════════════════════════════════════════════════════════════"
echo ""

# Give user time to read and cancel if needed
echo "Starting in 3 seconds... (Press Ctrl+C to cancel)"
sleep 1
echo "2..."
sleep 1
echo "1..."
sleep 1
echo ""

echo "🌳 Launching Behavior Tree..."
echo ""
echo "════════════════════════════════════════════════════════════════"
echo ""

# Launch the behavior tree using the launch file
ros2 launch ros_arduino_bridge behavior_tree_navigation.launch.py

echo ""
echo "════════════════════════════════════════════════════════════════"
echo ""
echo "🏁 Behavior tree mission ended."
echo ""
echo "Next steps:"
echo "  - Review mission completion in terminal output"
echo "  - Check robot final position in RViz"
echo "  - Modify waypoints in: behavior_tree/robot_navigation_bt.py"
echo "  - Create custom behaviors for your specific tasks"
echo ""
