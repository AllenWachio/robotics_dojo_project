#!/bin/bash
# Quick diagnostic script for mapping turn issues
# Run this WHILE the system is running to check if everything is configured correctly

echo "=========================================="
echo "SLAM Turn Issue Diagnostic Tool"
echo "=========================================="
echo ""

# Check if system is running
echo "1. Checking if core nodes are running..."
echo "   - Arduino Bridge:"
ros2 node list | grep -q "ros_arduino_bridge" && echo "     ✅ RUNNING" || echo "     ❌ NOT RUNNING"

echo "   - EKF Node:"
ros2 node list | grep -q "ekf_filter_node" && echo "     ✅ RUNNING" || echo "     ❌ NOT RUNNING - CRITICAL!"

echo "   - SLAM Toolbox:"
ros2 node list | grep -q "slam_toolbox" && echo "     ✅ RUNNING" || echo "     ❌ NOT RUNNING"

echo ""

# Check topics
echo "2. Checking critical topics..."
echo "   - /odom (raw encoders):"
ros2 topic list | grep -q "^/odom$" && echo "     ✅ EXISTS" || echo "     ❌ MISSING"

echo "   - /imu/data (IMU sensor):"
ros2 topic list | grep -q "/imu/data" && echo "     ✅ EXISTS" || echo "     ❌ MISSING - CRITICAL!"

echo "   - /odometry/filtered (fused):"
ros2 topic list | grep -q "/odometry/filtered" && echo "     ✅ EXISTS" || echo "     ❌ MISSING - CRITICAL!"

echo ""

# Check publishing rates
echo "3. Checking topic publishing rates (waiting 2 seconds)..."
ODOM_HZ=$(timeout 2 ros2 topic hz /odom 2>/dev/null | grep "average rate" | awk '{print $3}')
if [ ! -z "$ODOM_HZ" ]; then
    echo "   - /odom: ${ODOM_HZ} Hz (expected ~10 Hz)"
else
    echo "   - /odom: ❌ NOT PUBLISHING"
fi

IMU_HZ=$(timeout 2 ros2 topic hz /imu/data 2>/dev/null | grep "average rate" | awk '{print $3}')
if [ ! -z "$IMU_HZ" ]; then
    echo "   - /imu/data: ${IMU_HZ} Hz (expected ~5 Hz)"
else
    echo "   - /imu/data: ❌ NOT PUBLISHING"
fi

FILTERED_HZ=$(timeout 2 ros2 topic hz /odometry/filtered 2>/dev/null | grep "average rate" | awk '{print $3}')
if [ ! -z "$FILTERED_HZ" ]; then
    echo "   - /odometry/filtered: ${FILTERED_HZ} Hz (expected ~20 Hz)"
else
    echo "   - /odometry/filtered: ❌ NOT PUBLISHING"
fi

echo ""

# Check TF publishing (critical!)
echo "4. Checking TF publishing configuration..."
echo "   - Arduino Bridge publish_tf parameter:"
ARDUINO_TF=$(ros2 param get /ros_arduino_bridge publish_tf 2>/dev/null | grep "Boolean value is" | awk '{print $4}')
if [ "$ARDUINO_TF" == "False" ]; then
    echo "     ✅ DISABLED (correct - EKF handles TF)"
elif [ "$ARDUINO_TF" == "True" ]; then
    echo "     ❌ ENABLED - CRITICAL ERROR! This causes TF conflicts!"
else
    echo "     ⚠️  UNKNOWN - Check manually"
fi

echo ""

# Check SLAM subscription
echo "5. Checking what SLAM is subscribed to..."
SLAM_SUBS=$(ros2 topic info /odometry/filtered 2>/dev/null | grep -A 5 "Subscription count")
if echo "$SLAM_SUBS" | grep -q "slam_toolbox"; then
    echo "   ✅ SLAM is using /odometry/filtered (correct!)"
else
    echo "   ❌ SLAM NOT subscribed to /odometry/filtered - CRITICAL ERROR!"
    echo "      Check launch file remapping"
fi

echo ""

# Check IMU data quality
echo "6. Checking IMU data quality..."
echo "   Getting one IMU sample..."
IMU_SAMPLE=$(timeout 2 ros2 topic echo /imu/data --once 2>/dev/null)
if [ ! -z "$IMU_SAMPLE" ]; then
    ORIENT_W=$(echo "$IMU_SAMPLE" | grep -A 4 "orientation:" | grep "w:" | awk '{print $2}')
    GYRO_Z=$(echo "$IMU_SAMPLE" | grep -A 3 "angular_velocity:" | grep "z:" | awk '{print $2}')
    ACCEL_Z=$(echo "$IMU_SAMPLE" | grep -A 3 "linear_acceleration:" | grep "z:" | awk '{print $2}')
    
    echo "   - Orientation (w): $ORIENT_W"
    echo "   - Angular velocity (z): $GYRO_Z rad/s"
    echo "   - Linear accel (z): $ACCEL_Z m/s² (should be ~9.8 when stationary)"
    
    # Check if values are reasonable
    if [ "$ORIENT_W" == "0.0" ] || [ -z "$ORIENT_W" ]; then
        echo "   ⚠️  WARNING: Orientation seems to be zero - check Arduino IMU code"
    else
        echo "   ✅ IMU data looks valid"
    fi
else
    echo "   ❌ Could not get IMU sample - IMU not working!"
fi

echo ""

# Check odometry covariance
echo "7. Checking odometry covariance (needed for EKF)..."
ODOM_COV=$(timeout 2 ros2 topic echo /odom --once 2>/dev/null | grep -A 1 "pose:" | grep "covariance:")
if [ ! -z "$ODOM_COV" ]; then
    # Check if covariance is not all zeros
    if echo "$ODOM_COV" | grep -q "[1-9]"; then
        echo "   ✅ Odometry has covariance set"
    else
        echo "   ❌ Odometry covariance is all zeros - EKF won't work properly!"
    fi
else
    echo "   ❌ Could not check odometry covariance"
fi

echo ""
echo "=========================================="
echo "DIAGNOSTIC COMPLETE"
echo "=========================================="
echo ""

# Summary
echo "SUMMARY:"
echo ""

HAS_ERROR=0

if ! ros2 node list | grep -q "ekf_filter_node"; then
    echo "❌ CRITICAL: EKF node not running - sensor fusion disabled!"
    HAS_ERROR=1
fi

if ! ros2 topic list | grep -q "/odometry/filtered"; then
    echo "❌ CRITICAL: /odometry/filtered topic missing - EKF not working!"
    HAS_ERROR=1
fi

if [ "$ARDUINO_TF" == "True" ]; then
    echo "❌ CRITICAL: TF conflict - arduino_bridge publishing TF (should be False)"
    HAS_ERROR=1
fi

if ! echo "$SLAM_SUBS" | grep -q "slam_toolbox" 2>/dev/null; then
    echo "❌ CRITICAL: SLAM not using filtered odometry - mapping will break on turns!"
    HAS_ERROR=1
fi

if [ $HAS_ERROR -eq 0 ]; then
    echo "✅ All checks passed! System should work correctly."
    echo ""
    echo "If mapping still breaks during turns, try:"
    echo "  1. Increase SLAM minimum_travel_heading parameter"
    echo "  2. Check IMU calibration (rotate robot and watch /imu/data)"
    echo "  3. Tune EKF covariances"
else
    echo ""
    echo "⚠️  CRITICAL ISSUES FOUND - Fix these before mapping!"
    echo ""
    echo "Quick fixes:"
    echo "  1. Make sure you're using the new launch file:"
    echo "     ros2 launch ros_arduino_bridge slam_with_sensor_fusion.launch.py"
    echo ""
    echo "  2. Install robot_localization if missing:"
    echo "     sudo apt install ros-humble-robot-localization"
    echo ""
    echo "  3. Rebuild package after changes:"
    echo "     cd ~/ros2_ws && colcon build --packages-select ros_arduino_bridge"
fi

echo ""
