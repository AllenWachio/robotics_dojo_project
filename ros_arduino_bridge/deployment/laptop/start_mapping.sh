#!/bin/bash
echo "🚀 Starting Robot Base Station (SLAM Mapping Mode)..."
echo ""

# ROS2 Environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Networking
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

echo "🔍 Checking robot connection..."
echo "📡 Looking for robot topics..."

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

echo "🗺️  Starting SLAM processing and visualization..."
echo "🎮 Teleop keyboard will open for robot control"
echo ""

# Launch base station using full path
ros2 launch ~/ros2_ws/src/ros_arduino_bridge/deployment/laptop/laptop_base_station.launch.py

rm -f /tmp/robot_topics_check