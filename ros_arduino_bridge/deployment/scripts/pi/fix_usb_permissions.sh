#!/bin/bash

# Quick USB Permission Fix Script
# Run this if LiDAR has permission issues

echo "=== USB Permission Fix ==="
echo "Current USB device permissions:"
ls -la /dev/ttyUSB* 2>/dev/null

echo ""
echo "Fixing USB permissions..."

# Fix permissions for all USB devices
for device in /dev/ttyUSB*; do
    if [ -e "$device" ]; then
        echo "Fixing permissions for $device"
        sudo chmod 666 "$device"
    fi
done

echo ""
echo "Updated permissions:"
ls -la /dev/ttyUSB* 2>/dev/null

echo ""
echo "Checking power status:"
echo "CPU Temperature: $(cat /sys/class/thermal/thermal_zone0/temp | awk '{printf "%.1f°C", $1/1000}')"
if command -v vcgencmd >/dev/null 2>&1; then
    throttled=$(vcgencmd get_throttled)
    voltage=$(vcgencmd measure_volts)
    echo "Throttling status: $throttled"
    echo "Voltage: $voltage"
    
    # Check if voltage is too low
    voltage_num=$(echo $voltage | grep -o '[0-9.]*')
    if (( $(echo "$voltage_num < 1.1" | bc -l) )); then
        echo ""
        echo "⚠ WARNING: Voltage is low ($voltage)"
        echo "This may cause LiDAR communication issues."
        echo "Consider using a better power supply or powered USB hub."
    fi
    
    # Check throttling flags
    if [[ "$throttled" != "throttled=0x0" ]]; then
        echo ""
        echo "⚠ WARNING: System has been throttled"
        echo "This indicates power supply or thermal issues."
        echo "LiDAR may not work reliably."
    fi
fi

echo ""
echo "Permission fix complete. Try launching LiDAR again."