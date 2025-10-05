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

# Check devices first
    echo "Checking hardware connections..."
    
    arduino_device="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
    if [ -e "$arduino_device" ]; then
        echo "   ✓ Found Arduino device via device ID"
    else
        echo "   ✗ No Arduino device at expected device ID"
    fi
    
    lidar_device="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
    if [ -e "$lidar_device" ]; then
        echo "   ✓ Found LiDAR device via device ID"
    else
        echo "   ✗ No LiDAR device at expected device ID"
    fi
    
echo ""
echo "Starting hardware nodes..."

# Launch Pi hardware interface
ros2 launch ros_arduino_bridge pi_robot_hardware.launch.py