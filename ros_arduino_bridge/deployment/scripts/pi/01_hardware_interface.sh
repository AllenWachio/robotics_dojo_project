#!/bin/bash

# RASPBERRY PI SCRIPT - Hardware Interface
# This script runs on the Pi and handles all hardware communication
# Run this BEFORE starting laptop scripts

echo "Starting Raspberry Pi Hardware Interface..."
echo "This script manages Arduino and LiDAR hardware"
echo ""

# Source the workspace (Pi path structure)
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Build the workspace first
echo "Building workspace on Pi..."
cd ~/ros2_ws
colcon build --packages-select ros_arduino_bridge

if [ $? -eq 0 ]; then
    echo "Build successful! Starting hardware interface..."
    source ~/ros2_ws/install/setup.bash
    
    # Check devices first
    echo "Checking hardware connections..."
    
    if [ -e /dev/ttyUSB0 ]; then
        echo "   ✓ Found device at /dev/ttyUSB0 (likely Arduino)"
    else
        echo "   ✗ No device at /dev/ttyUSB0"
    fi
    
    if [ -e /dev/ttyUSB1 ]; then
        echo "   ✓ Found device at /dev/ttyUSB1 (likely LiDAR)"
    else
        echo "   ✗ No device at /dev/ttyUSB1"
    fi
    
    echo ""
    echo "Starting hardware nodes..."
    
    # Launch Pi hardware interface
    ros2 launch ros_arduino_bridge pi_robot_hardware.launch.py
    
else
    echo "Build failed! Please fix errors before running."
fi