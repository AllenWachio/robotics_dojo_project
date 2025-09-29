#!/bin/bash

# LAPTOP SCRIPT - Diagnostics and Testing

echo "ROS2 Arduino Bridge - Laptop Diagnostics"
echo "========================================"
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

# Check workspace build
echo "2. Checking workspace build..."
if [ -f ~/ros2_ws/install/setup.bash ]; then
    echo "   ✓ Workspace appears to be built"
else
    echo "   ✗ Workspace not built - run 'colcon build' in ~/ros2_ws"
fi
echo ""

# Check package
echo "3. Checking ros_arduino_bridge package..."
source /opt/ros/humble/setup.bash 2>/dev/null
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash 2>/dev/null
fi

if ros2 pkg list | grep -q ros_arduino_bridge; then
    echo "   ✓ ros_arduino_bridge package found"
else
    echo "   ✗ ros_arduino_bridge package not found"
fi
echo ""

# Check for saved maps
echo "4. Checking for saved maps..."
if [ -f ~/ros2_ws/maps/my_map.yaml ]; then
    echo "   ✓ Map file found: ~/ros2_ws/maps/my_map.yaml"
else
    echo "   ✗ No map file found - create one with mapping mode"
fi
echo ""

# Check network connectivity with Pi
echo "5. Network connectivity test..."
echo "   Enter Pi IP address (or press Enter to skip): "
read -r pi_ip

if [ -n "$pi_ip" ]; then
    if ping -c 1 "$pi_ip" &> /dev/null; then
        echo "   ✓ Can reach Pi at $pi_ip"
    else
        echo "   ✗ Cannot reach Pi at $pi_ip"
    fi
fi
echo ""

echo "Diagnostics complete!"