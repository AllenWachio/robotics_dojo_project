#!/bin/bash

# RASPBERRY PI SCRIPT - LiDAR Only
# This script launches just the LiDAR node
# Run this in a separate terminal after Arduino bridge is running

echo "Starting LiDAR Node..."
echo "Make sure Arduino bridge is already running in another terminal"
echo ""

# Source the workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Check LiDAR device and permissions
echo "Checking LiDAR connection..."

lidar_port=""
if [ -e /dev/ttyUSB1 ]; then
    echo "   ✓ Found device at /dev/ttyUSB1"
    if [ -w /dev/ttyUSB1 ]; then
        echo "   ✓ Write access confirmed"
        lidar_port="/dev/ttyUSB1"
    else
        echo "   ⚠ No write access to /dev/ttyUSB1"
        echo "   Trying to fix permissions..."
        sudo chmod 666 /dev/ttyUSB1
        if [ -w /dev/ttyUSB1 ]; then
            echo "   ✓ Permissions fixed"
            lidar_port="/dev/ttyUSB1"
        else
            echo "   ✗ Still no write access"
        fi
    fi
elif [ -e /dev/ttyUSB0 ]; then
    echo "   Checking /dev/ttyUSB0..."
    if [ -w /dev/ttyUSB0 ]; then
        echo "   ✓ Found LiDAR at /dev/ttyUSB0"
        lidar_port="/dev/ttyUSB0"
    fi
fi

if [ -z "$lidar_port" ]; then
    echo "   ✗ No accessible LiDAR device found"
    echo "   Available devices:"
    ls -la /dev/ttyUSB* 2>/dev/null || echo "   No /dev/ttyUSB* devices"
    exit 1
fi

echo "   Using LiDAR port: $lidar_port"
echo ""

# Check system resources
echo "System status:"
echo "   CPU Temp: $(cat /sys/class/thermal/thermal_zone0/temp | awk '{printf "%.1f°C", $1/1000}')"
echo "   Throttling: $(vcgencmd get_throttled)"
echo "   Voltage: $(vcgencmd measure_volts)"
echo ""

# Test different configurations
echo "Choose LiDAR configuration:"
echo "1) Standard (115200 baud)"
echo "2) High speed (256000 baud)" 
echo "3) Basic A1 config"
echo "4) Custom troubleshooting"
read -p "Selection (1-4): " choice

case $choice in
    1)
        echo "Starting with standard configuration..."
        ros2 run sllidar_ros2 sllidar_node --ros-args \
            -p channel_type:=serial \
            -p serial_port:=$lidar_port \
            -p serial_baudrate:=115200 \
            -p frame_id:=laser \
            -p inverted:=false \
            -p angle_compensate:=true
        ;;
    2)
        echo "Starting with high speed configuration..."
        ros2 run sllidar_ros2 sllidar_node --ros-args \
            -p channel_type:=serial \
            -p serial_port:=$lidar_port \
            -p serial_baudrate:=256000 \
            -p frame_id:=laser \
            -p inverted:=false \
            -p angle_compensate:=true
        ;;
    3)
        echo "Starting with A1 launch file..."
        ros2 launch sllidar_ros2 sllidar_a1_launch.py serial_port:=$lidar_port
        ;;
    4)
        echo "Custom troubleshooting mode..."
        echo "Trying minimal parameters..."
        ros2 run sllidar_ros2 sllidar_node --ros-args \
            -p channel_type:=serial \
            -p serial_port:=$lidar_port \
            -p serial_baudrate:=115200 \
            -p frame_id:=laser
        ;;
    *)
        echo "Invalid selection, using standard config..."
        ros2 run sllidar_ros2 sllidar_node --ros-args \
            -p channel_type:=serial \
            -p serial_port:=$lidar_port \
            -p serial_baudrate:=115200 \
            -p frame_id:=laser \
            -p inverted:=false \
            -p angle_compensate:=true
        ;;
esac