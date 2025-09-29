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
        echo "   ✗ Workspace not built - run 'colcon build'"
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

if [ -e /dev/ttyUSB0 ]; then
    echo "   Device permissions for /dev/ttyUSB0:"
    ls -la /dev/ttyUSB0
fi

if [ -e /dev/ttyUSB1 ]; then
    echo "   Device permissions for /dev/ttyUSB1:"
    ls -la /dev/ttyUSB1
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

echo "Setup check complete!"
echo ""
echo "If everything looks good, run: ./01_hardware_interface.sh"