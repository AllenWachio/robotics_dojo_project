#!/bin/bash

# LAPTOP SCRIPT - Mapping Mode
# This script runs on the laptop for creating maps
# The Pi should be running hardware nodes (see pi scripts)

echo "Starting Laptop Mapping Mode..."
echo "Make sure the Pi is running the hardware script first!"
echo ""

# Source the workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Build the workspace first
echo "Building workspace..."
cd ~/ros2_ws
colcon build --packages-select ros_arduino_bridge

if [ $? -eq 0 ]; then
    echo "Build successful! Starting mapping..."
    source ~/ros2_ws/install/setup.bash
    
    # Launch mapping on laptop
    ros2 launch ros_arduino_bridge laptop_base_station.launch.py
else
    echo "Build failed! Please fix errors before running."
fi