#!/bin/bash
echo "🔧 Robot Hardware Test"
echo "====================="
echo ""

# Setup environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash > /dev/null 2>&1

echo "🔍 Testing hardware connections..."
echo ""

# Test Arduino
echo "Arduino Test:"
if [ -e /dev/ttyUSB0 ]; then
    echo "✅ Arduino port: /dev/ttyUSB0"
    
    # Test Arduino communication
    python3 -c "
import serial
import time
try:
    print('   Testing communication...')
    ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=2)
    time.sleep(2)
    ser.write(b'e\r')
    ser.flush()
    response = ser.readline()
    ser.close()
    if response:
        print(f'✅ Arduino response: {response.decode().strip()}')
    else:
        print('❌ No response from Arduino')
except Exception as e:
    print(f'❌ Arduino error: {e}')
    "
else
    echo "❌ Arduino port: Not found at /dev/ttyUSB0"
    echo "Available ports:"
    ls /dev/ttyUSB* 2>/dev/null || echo "   None"
fi

echo ""

# Test LiDAR
echo "LiDAR Test:"
LIDAR_PORTS=("/dev/ttyUSB1" "/dev/ttyUSB2" "/dev/ttyUSB3")
LIDAR_FOUND=false

for port in "${LIDAR_PORTS[@]}"; do
    if [ -e "$port" ]; then
        echo "✅ LiDAR port found: $port"
        LIDAR_FOUND=true
        break
    fi
done

if ! $LIDAR_FOUND; then
    echo "❌ LiDAR port: Not found"
    echo "Available ports:"
    ls /dev/ttyUSB* 2>/dev/null || echo "   None"
fi

echo ""

# Test system resources
echo "System Resources:"
echo "   CPU Usage: $(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)%"
echo "   Memory: $(free -h | awk 'NR==2{printf "%.1f/%.1f GB (%.2f%%)", $3/1024/1024, $2/1024/1024, $3*100/$2}')"
echo "   Disk: $(df -h / | awk 'NR==2{print $3"/"$2" ("$5")"}')"
echo "   Temperature: $(vcgencmd measure_temp 2>/dev/null | cut -d= -f2 || echo "N/A")"

echo ""

# Test network
echo "Network Test:"
echo "   Pi IP: $(hostname -I | awk '{print $1}')"
echo "   ROS Domain: ${ROS_DOMAIN_ID:-0}"

# Test if ROS2 is working
echo ""
echo "ROS2 Test:"
if ros2 --version >/dev/null 2>&1; then
    echo "✅ ROS2 installation: $(ros2 --version)"
    
    # Quick topic test
    timeout 2s ros2 topic list >/dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "✅ ROS2 communication: Working"
    else
        echo "⚠️  ROS2 communication: Limited (normal if no nodes running)"
    fi
else
    echo "❌ ROS2 installation: Not working"
fi

echo ""
echo "🏁 Hardware test complete!"
echo ""
echo "💡 To start robot services: ./start_robot.sh"