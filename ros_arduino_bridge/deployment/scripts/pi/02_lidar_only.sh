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

# Use device ID approach for consistent device identification
lidar_port="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"

if [ -e "$lidar_port" ]; then
    echo "   ✓ Found LiDAR device via device ID"
    if [ -w "$lidar_port" ]; then
        echo "   ✓ Write access confirmed"
    else
        echo "   ⚠ No write access to device"
        echo "   Trying to fix permissions..."
        sudo chmod 666 "$lidar_port"
        if [ -w "$lidar_port" ]; then
            echo "   ✓ Permissions fixed"
        else
            echo "   ✗ Still no write access"
            exit 1
        fi
    fi
else
    echo "   ✗ LiDAR device not found at expected device ID"
    echo "   Available serial devices:"
    ls -la /dev/serial/by-id/ 2>/dev/null || echo "   No /dev/serial/by-id/ devices"
    echo "   Available USB devices:"
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