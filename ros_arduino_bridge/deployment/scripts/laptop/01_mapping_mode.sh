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

echo "Starting mapping..."

# Launch mapping on laptop
ros2 launch ros_arduino_bridge laptop_base_station.launch.py