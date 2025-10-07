#!/bin/bash

# RASPBERRY PI SCRIPT - Network Configuration Helper
# Use this to configure networking for distributed ROS2

echo "Raspberry Pi Network Configuration for ROS2"
echo "==========================================="
echo ""

# Get current IP
current_ip=$(hostname -I | awk '{print $1}')
echo "Current Pi IP address: $current_ip"
echo ""

# Check ROS_DOMAIN_ID
echo "ROS2 Configuration:"
if [ -n "$ROS_DOMAIN_ID" ]; then
    echo "   ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
else
    echo "   ROS_DOMAIN_ID: not set (default: 0)"
fi

if [ -n "$RMW_IMPLEMENTATION" ]; then
    echo "   RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
else
    echo "   RMW_IMPLEMENTATION: not set (default)"
fi
echo ""

# Network connectivity test
echo "Network Test:"
echo "Enter laptop IP address to test connectivity (or press Enter to skip):"
read -r laptop_ip

if [ -n "$laptop_ip" ]; then
    echo "Testing connection to laptop at $laptop_ip..."
    if ping -c 3 "$laptop_ip" &> /dev/null; then
        echo "   ✓ Can reach laptop at $laptop_ip"
    else
        echo "   ✗ Cannot reach laptop at $laptop_ip"
        echo "   Check network connection and firewall settings"
    fi
fi
echo ""

# Suggest environment setup
echo "Recommended environment setup for ~/.bashrc:"
echo "export ROS_DOMAIN_ID=0"
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"
echo ""

# Show current DDS discovery
echo "Current ROS2 nodes on network:"
source /opt/ros/humble/setup.bash 2>/dev/null
ros2 node list 2>/dev/null || echo "No ROS2 nodes currently running"
echo ""

echo "Configuration check complete!"
echo "Make sure both Pi and laptop have the same ROS_DOMAIN_ID"