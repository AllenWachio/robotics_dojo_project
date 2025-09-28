#!/bin/bash
# RASPBERRY PI SETUP SCRIPT
# Sets up the Raspberry Pi as the robot hardware interface

echo "🤖 RASPBERRY PI ROBOT SETUP"
echo "============================"
echo ""

# Get Pi information
PI_IP=$(hostname -I | awk '{print $1}')
echo "Raspberry Pi IP: $PI_IP"
echo ""

# Configure ROS2 networking
echo "🔧 Configuring ROS2 networking..."
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

# Make networking persistent
echo "# Distributed Robot Computing - Pi Robot Hardware" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc

# System optimizations for Pi
echo "⚡ Optimizing system performance..."

# Performance governor
echo 'performance' | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor > /dev/null

# Network optimizations
echo "net.core.rmem_max = 2147483647" | sudo tee -a /etc/sysctl.conf > /dev/null
echo "net.core.rmem_default = 2147483647" | sudo tee -a /etc/sysctl.conf > /dev/null
echo "net.core.netdev_max_backlog = 5000" | sudo tee -a /etc/sysctl.conf > /dev/null
sudo sysctl -p > /dev/null

# Create robot startup script
cat > ~/start_robot.sh << 'EOF'
#!/bin/bash
echo "🚀 Starting Robot Hardware Services..."
echo ""

# Setup ROS2 environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Networking
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# Check hardware connections
echo "🔍 Checking hardware connections..."
if [ -e /dev/ttyUSB0 ]; then
    echo "✅ Arduino found at /dev/ttyUSB0"
else
    echo "❌ Arduino not found at /dev/ttyUSB0"
    echo "Available devices:"
    ls /dev/ttyUSB* 2>/dev/null || echo "No USB serial devices found"
fi

if [ -e /dev/ttyUSB1 ]; then
    echo "✅ LiDAR found at /dev/ttyUSB1"  
    LIDAR_PORT="/dev/ttyUSB1"
else
    echo "⚠️  LiDAR not found at /dev/ttyUSB1, searching..."
    LIDAR_PORT=$(ls /dev/ttyUSB* 2>/dev/null | grep -v "/dev/ttyUSB0" | head -1)
    if [ -n "$LIDAR_PORT" ]; then
        echo "✅ LiDAR found at $LIDAR_PORT"
    else
        echo "❌ LiDAR not found"
        exit 1
    fi
fi

echo ""
echo "🤖 Launching robot hardware interface..."
cd ~/ros2_ws/src/ros_arduino_bridge/deployment/pi
ros2 launch . pi_robot_hardware.launch.py &



echo ""
echo "✅ Robot hardware services started!"
echo "📊 Publishing topics:"
echo "   /odom           - Robot odometry"
echo "   /scan           - LiDAR data"  
echo "   /tf             - Transform tree"
echo "   /joint_states   - Wheel positions"
echo "   /cmd_vel        - Motor commands (subscribed)"
echo ""
echo "🌐 Robot IP: $(hostname -I | awk '{print $1}')"
echo "🖥️  Connect laptop base station to access these topics"
echo ""
echo "Press Ctrl+C to stop robot services"

wait
EOF
chmod +x ~/start_robot.sh

# Create hardware test script
cat > ~/test_hardware.sh << 'EOF'
#!/bin/bash
echo "🔧 Robot Hardware Test"
echo "====================="
echo ""

# Setup environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash > /dev/null 2>&1

echo "Testing hardware connections..."
echo ""

# Test Arduino
if [ -e /dev/ttyUSB0 ]; then
    echo "✅ Arduino port: /dev/ttyUSB0"
    
    # Test Arduino communication
    python3 -c "
import serial
import time
try:
    ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=2)
    time.sleep(2)
    ser.write(b'e\r')
    ser.flush()
    response = ser.readline()
    ser.close()
    if response:
        print('✅ Arduino communication: OK')
        print(f'   Response: {response.decode().strip()}')
    else:
        print('❌ Arduino communication: No response')
except Exception as e:
    print(f'❌ Arduino communication: {e}')
    "
else
    echo "❌ Arduino port: Not found"
fi

echo ""

# Test LiDAR
LIDAR_PORTS=(/dev/ttyUSB1 /dev/ttyUSB2 /dev/ttyUSB3)
LIDAR_FOUND=false

for port in "${LIDAR_PORTS[@]}"; do
    if [ -e "$port" ]; then
        echo "✅ LiDAR port: $port"
        LIDAR_FOUND=true
        break
    fi
done

if [ "$LIDAR_FOUND" = false ]; then
    echo "❌ LiDAR port: Not found"
fi

echo ""

# Test system resources
echo "System resources:"
echo "   CPU cores: $(nproc)"
echo "   Memory: $(free -h | awk '/^Mem:/ {print $2}') total, $(free -h | awk '/^Mem:/ {print $7}') available"
echo "   Storage: $(df -h / | awk 'NR==2 {print $4}') free"
echo ""

# Test network
echo "Network configuration:"
echo "   IP address: $(hostname -I | awk '{print $1}')"
echo "   Network interface: $(ip route | grep default | awk '{print $5}')"
EOF
chmod +x ~/test_hardware.sh

# Create system monitor script
cat > ~/monitor_robot.sh << 'EOF'
#!/bin/bash
echo "📊 Robot System Monitor"
echo ""

# Setup environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash > /dev/null 2>&1
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

while true; do
    clear
    echo "📊 Robot System Monitor - $(date)"
    echo "=================================="
    echo ""
    
    # System stats
    echo "💻 System:"
    echo "   CPU: $(cat /proc/loadavg | awk '{print $1}')"
    echo "   Memory: $(free | awk '/^Mem:/ {printf "%.1f%%", $3/$2 * 100.0}')"
    echo "   Temperature: $(vcgencmd measure_temp | cut -d'=' -f2)"
    echo ""
    
    # ROS2 nodes
    echo "🤖 ROS2 Nodes:"
    timeout 3s ros2 node list 2>/dev/null | sed 's/^/   /' || echo "   No nodes detected"
    echo ""
    
    # Topic rates
    echo "📡 Topic Rates:"
    for topic in /odom /scan /tf; do
        rate=$(timeout 3s ros2 topic hz $topic 2>/dev/null | grep "average rate" | awk '{print $3}' | head -1)
        if [ -n "$rate" ]; then
            echo "   $topic: ${rate} Hz"
        else
            echo "   $topic: No data"
        fi
    done
    
    echo ""
    echo "Press Ctrl+C to exit"
    sleep 5
done
EOF
chmod +x ~/monitor_robot.sh

echo ""
echo "✅ Raspberry Pi robot setup complete!"
echo ""
echo "📋 Next Steps:"
echo "1. Test hardware:    ./test_hardware.sh"
echo "2. Start robot:      ./start_robot.sh"
echo "3. Monitor system:   ./monitor_robot.sh"
echo ""
echo "📂 Files created:"
echo "   ~/start_robot.sh     - Start robot hardware services"
echo "   ~/test_hardware.sh   - Test Arduino and LiDAR connections"
echo "   ~/monitor_robot.sh   - Real-time system monitoring"
echo ""
echo "🌐 Network Configuration:"
echo "   Robot IP: $PI_IP"
echo "   ROS Domain: 0"
echo ""
echo "📡 The robot will publish topics that your laptop can automatically discover!"