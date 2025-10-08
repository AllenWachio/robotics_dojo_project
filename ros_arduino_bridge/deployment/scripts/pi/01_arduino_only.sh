#!/bin/bash

# RASPBERRY PI SCRIPT - Arduino Bridge with EKF Sensor Fusion
# This script launches the Arduino interface with EKF sensor fusion
# Run this in one terminal, then launch LiDAR separately

echo "Starting Arduino Bridge with EKF Sensor Fusion..."
echo "This fuses wheel encoders + IMU for accurate odometry"
echo ""

# Source the workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Check Arduino device using device ID
    echo "Checking Arduino connection..."
    ARDUINO_PORT="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
    
    if [ -e "$ARDUINO_PORT" ]; then
        echo "   ✓ Found Arduino via device ID"
        if [ -w "$ARDUINO_PORT" ]; then
            echo "   ✓ Write access confirmed"
        else
            echo "   ⚠ No write access to Arduino device"
            echo "   Trying to fix permissions..."
            sudo chmod 666 "$ARDUINO_PORT"
            if [ -w "$ARDUINO_PORT" ]; then
                echo "   ✓ Permissions fixed"
            else
                echo "   ✗ Still no write access"
                exit 1
            fi
        fi
    else
        echo "   ✗ Arduino device not found at expected device ID"
        echo "   Available serial devices:"
        ls -la /dev/serial/by-id/ 2>/dev/null || echo "   No /dev/serial/by-id/ devices"
        exit 1
    fi
    
echo ""
echo "Starting Arduino bridge nodes with EKF..."
echo "   - Arduino bridge publishes: /odom (raw), /imu/data"
echo "   - EKF publishes: /odometry/filtered (fused), TF transform"
echo ""

ros2 launch ros_arduino_bridge arduino_only.launch.py \
    arduino_port:="$ARDUINO_PORT" \
    use_ekf:=true \
    publish_tf:=false