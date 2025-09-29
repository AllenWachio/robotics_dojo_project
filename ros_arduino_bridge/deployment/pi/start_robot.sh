#!/bin/bash
echo "🤖 Starting Robot Hardware Services..."
echo ""

# Setup ROS2 environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Networking
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

echo "🔍 Checking hardware connections..."

# Check Arduino
if [ -e /dev/ttyUSB0 ]; then
    echo "✅ Arduino found at /dev/ttyUSB0"
    ARDUINO_PORT="/dev/ttyUSB0"
else
    echo "❌ Arduino not found at /dev/ttyUSB0"
    echo "Available USB devices:"
    ls /dev/ttyUSB* 2>/dev/null || echo "   No USB serial devices found"
    exit 1
fi

# Check LiDAR
if [ -e /dev/ttyUSB1 ]; then
    echo "✅ LiDAR found at /dev/ttyUSB1"  
    LIDAR_PORT="/dev/ttyUSB1"
else
    echo "⚠️  LiDAR not found at /dev/ttyUSB1, searching other ports..."
    LIDAR_PORT=$(ls /dev/ttyUSB* 2>/dev/null | grep -v "/dev/ttyUSB0" | head -1)
    if [ -n "$LIDAR_PORT" ]; then
        echo "✅ LiDAR found at $LIDAR_PORT"
    else
        echo "❌ LiDAR not found on any USB port"
        exit 1
    fi
fi

echo ""
echo "🧪 Testing hardware communication..."

# Test Arduino communication
python3 -c "
import serial
import time
try:
    ser = serial.Serial('$ARDUINO_PORT', 57600, timeout=2)
    time.sleep(2)
    ser.write(b'e\r')
    ser.flush()
    response = ser.readline()
    ser.close()
    if response:
        print('✅ Arduino communication: OK')
        print(f'   Encoder response: {response.decode().strip()}')
    else:
        print('❌ Arduino communication: No response')
        exit(1)
except Exception as e:
    print(f'❌ Arduino communication error: {e}')
    exit(1)
" || exit 1

echo ""
echo "🚀 Launching robot hardware interface..."

# Launch hardware using full path (not ros2 launch .)
ros2 launch ~/ros2_ws/src/ros_arduino_bridge/deployment/pi/pi_robot_hardware.launch.py arduino_port:="$ARDUINO_PORT" &

HARDWARE_PID=$!

echo ""
echo "✅ Robot hardware services started!"
echo "📊 Publishing topics:"
echo "   /odom           - Robot odometry from encoders"
echo "   /scan           - LiDAR laser scan data"  
echo "   /tf             - Robot transform tree"
echo "   /joint_states   - Wheel joint positions"
echo ""
echo "📡 Subscribes to:"
echo "   /cmd_vel        - Velocity commands for motors"
echo ""
echo "🌐 Robot IP: $(hostname -I | awk '{print $1}')"
echo "🔗 ROS Domain: $ROS_DOMAIN_ID"
echo ""
echo "🖥️  Connect laptop base station to access these topics"
echo "📋 On laptop run: ./start_mapping.sh (for mapping)"
echo "📋           or: ./start_navigation.sh (for navigation)"
echo ""
echo "Press Ctrl+C to stop robot services"

# Wait for the background process and handle shutdown gracefully
trap 'echo ""; echo "🛑 Stopping robot services..."; kill $HARDWARE_PID 2>/dev/null; exit 0' INT TERM

wait $HARDWARE_PID