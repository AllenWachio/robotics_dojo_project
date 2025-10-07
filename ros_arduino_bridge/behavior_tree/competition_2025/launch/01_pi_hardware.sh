#!/bin/bash

# ╔════════════════════════════════════════════════════════════════╗
# ║           🤖 RASPBERRY PI - COMPETITION HARDWARE 🤖            ║
# ║                  (Run this on the Raspberry Pi)               ║
# ╚════════════════════════════════════════════════════════════════╝
#
# This script runs on the Raspberry Pi and handles ALL hardware:
# - Arduino bridge (motors, encoders, IMU, servos, RGB sensor)
# - LiDAR sensor
# - Pi Camera (low-level capture and publishing)
# - Robot state publisher (TF tree)
#
# Heavy processing (Nav2, ML, behavior tree) runs on laptop!

echo "════════════════════════════════════════════════════════════════"
echo "    🤖 STARTING RASPBERRY PI HARDWARE FOR COMPETITION 🤖"
echo "════════════════════════════════════════════════════════════════"
echo ""

# Source workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Check Arduino device
echo "🔍 Checking Arduino connection..."
ARDUINO_PORT="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"

if [ -e "$ARDUINO_PORT" ]; then
    echo "   ✅ Arduino found"
    if [ ! -w "$ARDUINO_PORT" ]; then
        echo "   ⚠️  Fixing permissions..."
        sudo chmod 666 "$ARDUINO_PORT"
    fi
else
    echo "   ❌ Arduino NOT found!"
    echo "   Available devices:"
    ls -la /dev/serial/by-id/ 2>/dev/null || echo "   (none)"
    exit 1
fi

# Check LiDAR device
echo "🔍 Checking LiDAR connection..."
LIDAR_PORT="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"

if [ -e "$LIDAR_PORT" ]; then
    echo "   ✅ LiDAR found"
else
    echo "   ⚠️  LiDAR NOT found (navigation will fail!)"
fi

# Check camera
echo "🔍 Checking Pi Camera..."
if [ -e "/dev/video0" ]; then
    echo "   ✅ Camera found at /dev/video0"
else
    echo "   ⚠️  Camera NOT found (color detection will fail!)"
fi

echo ""
echo "════════════════════════════════════════════════════════════════"
echo "   📡 PI WILL PUBLISH (laptop automatically subscribes):"
echo "════════════════════════════════════════════════════════════════"
echo "   /odom                    - Wheel odometry"
echo "   /imu/data                - IMU measurements"
echo "   /scan                    - LiDAR scan data"
echo "   /camera/image_raw        - Camera feed (uncompressed)"
echo "   /camera/image_raw/compressed  - Camera feed (compressed)"
echo "   /color_sensor/rgb        - RGB sensor readings"
echo "   /camera_servo/command    - Servo control"
echo "   /tipper_servo/command    - Tipper control"
echo "   /stepper/command         - Stepper motor control"
echo "   /conveyor/command        - Conveyor belt control"
echo "   TF tree (robot transforms)"
echo ""
echo "════════════════════════════════════════════════════════════════"
echo ""

echo "🚀 Starting hardware nodes..."
echo ""

# Launch all Pi hardware
ros2 launch ros_arduino_bridge pi_competition_hardware.launch.py \
    arduino_port:="$ARDUINO_PORT" \
    lidar_port:="$LIDAR_PORT"

echo ""
echo "════════════════════════════════════════════════════════════════"
echo "   🛑 Pi hardware stopped"
echo "════════════════════════════════════════════════════════════════"
