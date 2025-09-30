#!/bin/bash

# RASPBERRY PI SCRIPT - Arduino Bridge Only
# This script launches just the Arduino interface and robot state publisher
# Run this in one terminal, then launch LiDAR separately

echo "Starting Arduino Bridge and Robot State Publisher..."
echo "This handles only Arduino communication and TF publishing"
echo ""

# Source the workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Build if needed
echo "Building workspace..."
cd ~/ros2_ws
colcon build --packages-select ros_arduino_bridge

if [ $? -eq 0 ]; then
    echo "Build successful! Starting Arduino bridge..."
    source ~/ros2_ws/install/setup.bash
    
    # Check Arduino device
    echo "Checking Arduino connection..."
    if [ -e /dev/ttyUSB0 ]; then
        echo "   ✓ Found Arduino at /dev/ttyUSB0"
        if [ -w /dev/ttyUSB0 ]; then
            echo "   ✓ Write access confirmed"
        else
            echo "   ⚠ Write access issue"
        fi
    else
        echo "   ✗ Arduino not found at /dev/ttyUSB0"
        echo "   Checking /dev/ttyUSB1..."
        if [ -e /dev/ttyUSB1 ] && [ -w /dev/ttyUSB1 ]; then
            echo "   ✓ Found Arduino at /dev/ttyUSB1 (will use this)"
            export ARDUINO_PORT="/dev/ttyUSB1"
        else
            echo "   ✗ No accessible Arduino device found"
            exit 1
        fi
    fi
    
    echo ""
    echo "Starting Arduino bridge nodes..."
    
    # Launch only Arduino components using package launch file
    ros2 launch ros_arduino_bridge arduino_only.launch.py arduino_port:=${ARDUINO_PORT:-/dev/ttyUSB0}
    
else
    echo "Build failed! Please fix errors before running."
    exit 1
fi