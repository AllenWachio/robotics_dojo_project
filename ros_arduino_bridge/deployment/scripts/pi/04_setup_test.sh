#!/bin/bash

# RASPBERRY PI SCRIPT - Setup and Test Hardware
# Use this to test Arduino and LiDAR connections

echo "Raspberry Pi Hardware Setup and Test"
echo "===================================="
echo ""

# Check ROS2 installation
echo "1. Checking ROS2 installation..."
if command -v ros2 &> /dev/null; then
    echo "   ✓ ROS2 is installed"
    ros2 --version
else
    echo "   ✗ ROS2 not found"
fi
echo ""

# Check workspace
echo "2. Checking workspace..."
if [ -d ~/ros2_ws ]; then
    echo "   ✓ Workspace directory exists"
    if [ -f ~/ros2_ws/install/setup.bash ]; then
        echo "   ✓ Workspace is built"
    else
        echo "   ✗ Workspace not built - please build the workspace"
    fi
else
    echo "   ✗ Workspace directory not found"
fi
echo ""

# Check USB devices
echo "3. Checking USB devices..."
echo "   Available USB devices:"
ls -la /dev/ttyUSB* 2>/dev/null || echo "   No USB devices found"
echo ""

# Device permissions are checked above using device IDs

lidar_device_id="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
arduino_device_id="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"

if [ -e "$arduino_device_id" ]; then
    echo "   Device permissions for Arduino (via device ID):"
    ls -la "$arduino_device_id"
fi

if [ -e "$lidar_device_id" ]; then
    echo "   Device permissions for LiDAR (via device ID):"
    ls -la "$lidar_device_id"
fi
echo ""

# Check user groups (for device access)
echo "4. Checking user permissions..."
if groups | grep -q dialout; then
    echo "   ✓ User is in 'dialout' group"
else
    echo "   ✗ User not in 'dialout' group"
    echo "   Run: sudo usermod -a -G dialout $USER"
    echo "   Then logout and login again"
fi
echo ""

# Test basic ROS2 functionality
echo "5. Testing ROS2 functionality..."
source /opt/ros/humble/setup.bash 2>/dev/null
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash 2>/dev/null
fi

echo "   Available ROS2 packages:"
ros2 pkg list | grep -E "(arduino|bridge|sllidar)" || echo "   No relevant packages found"
echo ""

# Test LiDAR communication
echo "6. Testing LiDAR communication..."
lidar_device="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
if [ -e "$lidar_device" ]; then
    echo "   Testing LiDAR connection via device ID..."
    # Quick test to see if device responds
    timeout 3s python3 -c "
import serial
import time
try:
    ser = serial.Serial('$lidar_device', 115200, timeout=1)
    print('   ✓ LiDAR device opens successfully')
    ser.close()
except Exception as e:
    print(f'   ✗ LiDAR connection failed: {e}')
" 2>/dev/null || echo "   ⚠ LiDAR test skipped (no python3/serial)"
else
    echo "   ✗ No LiDAR device found at expected device ID"
    echo "   Available serial devices:"
    ls -la /dev/serial/by-id/ 2>/dev/null || echo "   No /dev/serial/by-id/ devices"
fi
echo ""

echo "Setup check complete!"
echo ""
echo "If everything looks good, run: ./01_hardware_interface.sh"
echo ""
echo "If LiDAR fails to start, try:"
echo "  - Check device permissions: ls -la /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
echo "  - Power cycle the LiDAR"
echo "  - Try different USB port"