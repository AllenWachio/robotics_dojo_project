#!/bin/bash

# LAPTOP SCRIPT - Navigation Mode
# This script runs on the laptop for autonomous navigation using saved maps
# The Pi should be running hardware nodes (see pi scripts)

echo "Starting Laptop Navigation Mode..."
echo "Make sure you have saved a map from mapping mode first!"
echo "Make sure the Pi is running the hardware script!"
echo ""

# Source the workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Build the workspace first
echo "Building workspace..."
cd ~/ros2_ws
colcon build --packages-select ros_arduino_bridge

if [ $? -eq 0 ]; then
    echo "Build successful! Starting navigation..."
    source ~/ros2_ws/install/setup.bash
    
    # Launch navigation on laptop
    ros2 launch ros_arduino_bridge laptop_navigation.launch.py
else
    echo "Build failed! Please fix errors before running."
fi