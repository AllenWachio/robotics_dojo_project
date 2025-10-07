#!/bin/bash

################################################################################
# Camera Diagnostics Script
# 
# This script checks what video devices and formats are available on your Pi
################################################################################

echo "╔════════════════════════════════════════════════════════════════╗"
echo "║               Camera Diagnostics                              ║"
echo "╚════════════════════════════════════════════════════════════════╝"
echo ""

# Check for v4l-utils
if ! command -v v4l2-ctl &> /dev/null; then
    echo "❌ v4l2-ctl not found. Installing..."
    echo "Run: sudo apt-get install v4l-utils"
    exit 1
fi

echo "═══ Available Video Devices ═══"
echo ""
ls -la /dev/video* 2>/dev/null || echo "No video devices found!"
echo ""

echo "═══ Camera Detection ═══"
echo ""
vcgencmd get_camera 2>/dev/null || echo "vcgencmd not available"
echo ""

# Check each video device
for device in /dev/video*; do
    if [ -e "$device" ]; then
        echo "════════════════════════════════════════════════════════════════"
        echo "Device: $device"
        echo "════════════════════════════════════════════════════════════════"
        echo ""
        
        echo "--- Device Info ---"
        v4l2-ctl --device=$device --all 2>&1 | head -20
        echo ""
        
        echo "--- Supported Formats ---"
        v4l2-ctl --device=$device --list-formats-ext 2>&1
        echo ""
        echo ""
    fi
done

echo "════════════════════════════════════════════════════════════════"
echo "Diagnosis Complete!"
echo "════════════════════════════════════════════════════════════════"
echo ""
echo "Look for:"
echo "  1. Supported pixel formats (YUYV, MJPG, RGB3, etc.)"
echo "  2. Supported resolutions for each format"
echo "  3. Which /dev/video device has usable formats"
echo ""
