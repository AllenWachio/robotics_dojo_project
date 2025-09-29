#!/bin/bash
echo "🧭 Starting Robot Navigation System..."
echo ""

# ROS2 Environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Networking
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# Get map file
MAP_FILE="$1"
MAP_DIR="$HOME/robot_maps"

if [ -z "$MAP_FILE" ]; then
    echo "📋 Available maps:"
    if [ -d "$MAP_DIR" ] && ls "$MAP_DIR"/*.yaml &> /dev/null; then
        ls "$MAP_DIR"/*.yaml 2>/dev/null | sed 's|.*/||; s|\.yaml||' | awk '{print "   " $1}'
    else
        echo "   (No maps found in $MAP_DIR)"
        echo ""
        echo "💡 Create maps first using: ./start_mapping.sh"
        exit 1
    fi
    echo ""
    read -p "Enter map name (without .yaml): " MAP_FILE
fi

if [ -z "$MAP_FILE" ]; then
    MAP_FILE="robot_map"
fi

# Add .yaml extension if not present
if [[ ! "$MAP_FILE" == *.yaml ]]; then
    MAP_FILE="${MAP_FILE}.yaml"
fi

# Check if map exists
if [ ! -f "$MAP_DIR/$MAP_FILE" ]; then
    echo "❌ Map $MAP_DIR/$MAP_FILE not found!"
    echo "📋 Available maps:"
    if [ -d "$MAP_DIR" ]; then
        ls "$MAP_DIR"/*.yaml 2>/dev/null | sed 's|.*/||; s|\.yaml||' | awk '{print "   " $1}' || echo "   (No maps found)"
    fi
    exit 1
fi

echo "🗺️  Loading map: $MAP_FILE"
echo "🤖 Checking robot connection..."

# Check if robot topics are available
timeout 5s ros2 topic list 2>/dev/null | grep -E "(odom|scan|tf)" > /tmp/robot_topics_check
if [ -s /tmp/robot_topics_check ]; then
    echo "✅ Robot topics detected:"
    cat /tmp/robot_topics_check
    echo ""
else
    echo "⚠️  Robot topics not detected. Make sure robot is running:"
    echo "   1. SSH to Pi: ssh pi@<robot_ip>"  
    echo "   2. Start robot: ~/start_robot.sh"
    echo ""
    echo "Continuing anyway..."
fi

echo "🧭 Starting autonomous navigation..."
echo ""
echo "📖 Navigation Instructions:"
echo "1. Set initial pose using '2D Pose Estimate' tool in RViz"
echo "2. Set navigation goal using '2D Nav Goal' tool"
echo "3. Robot will plan path and navigate autonomously"
echo ""

# Launch navigation system using full path
ros2 launch ~/ros2_ws/src/ros_arduino_bridge/deployment/laptop/laptop_navigation.launch.py map_file:="$MAP_FILE"

rm -f /tmp/robot_topics_check