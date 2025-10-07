#!/bin/bash

# RASPBERRY PI SCRIPT - LiDAR Troubleshooting
# Use this to diagnose and fix LiDAR issues

echo "LiDAR Troubleshooting for SLLIDAR"
echo "================================="
echo ""

# Check if LiDAR device exists
echo "1. Checking LiDAR device..."
lidar_device="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
if [ -e "$lidar_device" ]; then
    echo "   ✓ LiDAR device found via device ID"
    ls -la "$lidar_device"
else
    echo "   ✗ No LiDAR device at expected device ID"
    echo "   Available serial devices:"
    ls -la /dev/serial/by-id/ 2>/dev/null || echo "   No /dev/serial/by-id/ devices"
    echo "   Available USB devices:"
    ls -la /dev/ttyUSB* 2>/dev/null || echo "   No USB devices found"
fi
echo ""

# Check permissions
echo "2. Checking device permissions..."
if groups | grep -q dialout; then
    echo "   ✓ User is in dialout group"
else
    echo "   ✗ User not in dialout group"
    echo "   Run: sudo usermod -a -G dialout $USER && logout/login"
fi
echo ""

# Try to reset LiDAR
echo "3. LiDAR reset options..."
echo "   To reset LiDAR, try one of these:"
echo "   a) Power cycle: Unplug and reconnect USB"
echo "   b) Try different USB port"
echo "   c) Restart Pi: sudo reboot"
echo ""

# Check if LiDAR process is running
echo "4. Checking for running LiDAR processes..."
if pgrep -f "sllidar" > /dev/null; then
    echo "   ⚠ LiDAR process already running:"
    pgrep -f "sllidar" | while read pid; do
        echo "     PID $pid: $(ps -p $pid -o comm=)"
    done
    echo "   You may need to kill existing processes:"
    echo "   pkill -f sllidar"
else
    echo "   ✓ No conflicting LiDAR processes found"
fi
echo ""

# Test basic serial communication
echo "5. Testing basic serial communication..."
if [ -e "$lidar_device" ] && command -v python3 &> /dev/null; then
    echo "   Testing serial communication..."
    python3 -c "
import serial
import time
try:
    print('   Opening serial connection...')
    ser = serial.Serial('$lidar_device', 115200, timeout=2)
    time.sleep(1)
    print('   ✓ Serial connection successful')
    ser.close()
    print('   ✓ Serial connection closed properly')
except PermissionError:
    print('   ✗ Permission denied - check user groups')
except serial.SerialException as e:
    print(f'   ✗ Serial error: {e}')
except Exception as e:
    print(f'   ✗ Unexpected error: {e}')
"
else
    echo "   ⚠ Cannot test - device missing or python3 not available"
fi
echo ""

echo "Troubleshooting complete!"
echo ""
echo "Common LiDAR error 80008002 solutions:"
echo "1. Power cycle the LiDAR (unplug/reconnect USB)"
echo "2. Try a different USB port"
echo "3. Check if another process is using the device"
echo "4. Verify device permissions (dialout group)"
echo "5. Restart the Raspberry Pi"
echo ""
echo "After fixing, try running: ./01_hardware_interface.sh"