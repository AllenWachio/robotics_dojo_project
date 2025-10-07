#!/bin/bash
# TF Diagnostic Script
# Run this script to check for TF conflicts and verify proper configuration

echo "========================================"
echo "ROS2 TF Tree Diagnostic Tool"
echo "========================================"
echo ""

# Check if ROS2 is sourced
if ! command -v ros2 &> /dev/null; then
    echo "❌ ERROR: ROS2 not found. Please source your workspace:"
    echo "   source /opt/ros/humble/setup.bash"
    echo "   source ~/ros2_ws/install/setup.bash"
    exit 1
fi

echo "✅ ROS2 environment detected"
echo ""

# Check if any nodes are running
echo "📊 Checking active ROS2 nodes..."
NODES=$(ros2 node list 2>/dev/null)
if [ -z "$NODES" ]; then
    echo "⚠️  WARNING: No ROS2 nodes running!"
    echo "   Start your launch file first, then run this script."
    exit 1
fi
echo "$NODES"
echo ""

# Check for TF publishers
echo "📡 Checking TF publishers..."
TF_PUBLISHERS=$(ros2 topic info /tf 2>/dev/null | grep "Publisher count:" -A 10)
echo "$TF_PUBLISHERS"
echo ""

# Look for potential conflicts
echo "🔍 Checking for odom→base_link TF conflicts..."
ODOM_PUBLISHERS=$(ros2 topic echo /tf --once 2>/dev/null | grep -A 5 "frame_id: odom" | grep "child_frame_id: base_link")
if [ ! -z "$ODOM_PUBLISHERS" ]; then
    echo "✅ odom→base_link transform is being published"
else
    echo "⚠️  WARNING: No odom→base_link transform detected!"
fi
echo ""

# Check for arduino_bridge
echo "🤖 Checking Arduino Bridge configuration..."
if ros2 node list 2>/dev/null | grep -q "ros_arduino_bridge"; then
    echo "✅ Arduino Bridge node is running"
    
    # Try to get publish_tf parameter
    PUBLISH_TF=$(ros2 param get /ros_arduino_bridge publish_tf 2>/dev/null)
    echo "   publish_tf parameter: $PUBLISH_TF"
    
    # Check if using EKF
    if ros2 node list 2>/dev/null | grep -q "ekf_filter_node"; then
        echo "   EKF detected: Arduino Bridge should have publish_tf=False"
        if echo "$PUBLISH_TF" | grep -q "False"; then
            echo "   ✅ CORRECT: publish_tf is False (EKF handles TF)"
        else
            echo "   ❌ ERROR: publish_tf should be False when using EKF!"
            echo "   This causes TF conflicts!"
        fi
    else
        echo "   No EKF detected: Arduino Bridge should have publish_tf=True"
        if echo "$PUBLISH_TF" | grep -q "True"; then
            echo "   ✅ CORRECT: publish_tf is True (no EKF)"
        else
            echo "   ⚠️  WARNING: publish_tf should be True without EKF!"
        fi
    fi
else
    echo "❌ Arduino Bridge node NOT running"
fi
echo ""

# Check for EKF
echo "🧮 Checking EKF Sensor Fusion..."
if ros2 node list 2>/dev/null | grep -q "ekf_filter_node"; then
    echo "✅ EKF filter node is running"
    
    # Check if it's publishing TF
    EKF_PUBLISH_TF=$(ros2 param get /ekf_filter_node publish_tf 2>/dev/null)
    echo "   publish_tf parameter: $EKF_PUBLISH_TF"
    
    if echo "$EKF_PUBLISH_TF" | grep -q "True"; then
        echo "   ✅ CORRECT: EKF is publishing TF"
    else
        echo "   ❌ ERROR: EKF should publish TF for sensor fusion!"
    fi
    
    # Check input topics
    echo "   Checking EKF inputs..."
    if ros2 topic list 2>/dev/null | grep -q "/odom"; then
        echo "   ✅ /odom topic available (encoder input)"
    fi
    if ros2 topic list 2>/dev/null | grep -q "/imu/data"; then
        echo "   ✅ /imu/data topic available (IMU input)"
    fi
    if ros2 topic list 2>/dev/null | grep -q "/odometry/filtered"; then
        echo "   ✅ /odometry/filtered topic available (EKF output)"
    fi
else
    echo "⚠️  EKF filter node NOT running (raw odometry mode)"
fi
echo ""

# Check robot_state_publisher
echo "🦾 Checking Robot State Publisher..."
if ros2 node list 2>/dev/null | grep -q "robot_state_publisher"; then
    echo "✅ Robot State Publisher is running"
    
    # Check if URDF is loaded
    ROBOT_DESC=$(ros2 param get /robot_state_publisher robot_description 2>/dev/null | head -n 5)
    if [ ! -z "$ROBOT_DESC" ]; then
        echo "   ✅ Robot description (URDF) is loaded"
    else
        echo "   ❌ ERROR: Robot description not loaded!"
    fi
else
    echo "❌ Robot State Publisher NOT running"
    echo "   Wheels won't appear in RViz without this!"
fi
echo ""

# Check SLAM
echo "🗺️  Checking SLAM Toolbox..."
if ros2 node list 2>/dev/null | grep -q "slam_toolbox"; then
    echo "✅ SLAM Toolbox is running"
else
    echo "⚠️  SLAM Toolbox NOT running (no mapping)"
fi
echo ""

# Generate TF tree
echo "🌳 Generating TF tree visualization..."
if ros2 run tf2_tools view_frames 2>/dev/null; then
    echo "✅ TF tree saved to: frames.pdf"
    echo "   Open this file to visualize the transform tree"
else
    echo "❌ Could not generate TF tree"
fi
echo ""

# Summary
echo "========================================"
echo "📋 SUMMARY"
echo "========================================"
echo ""
echo "Configuration Status:"
echo ""

# Determine mode
if ros2 node list 2>/dev/null | grep -q "ekf_filter_node"; then
    echo "Mode: SENSOR FUSION (EKF)"
    echo ""
    echo "Expected Configuration:"
    echo "  - Arduino Bridge: publish_tf = False ✓"
    echo "  - EKF: publish_tf = True ✓"
    echo "  - TF Flow: EKF publishes odom→base_link"
    echo ""
    echo "Launch file used should be:"
    echo "  - slam_with_sensor_fusion.launch.py"
    echo "  - arduino_bridge.py (with sensor_fusion.launch.py)"
else
    echo "Mode: RAW ODOMETRY (No EKF)"
    echo ""
    echo "Expected Configuration:"
    echo "  - Arduino Bridge: publish_tf = True ✓"
    echo "  - No EKF needed"
    echo "  - TF Flow: Arduino Bridge publishes odom→base_link"
    echo ""
    echo "Launch file used should be:"
    echo "  - test_teleop.launch.py"
    echo "  - full_slam_test.launch.py"
fi

echo ""
echo "Next Steps:"
echo "1. Open frames.pdf to verify TF tree structure"
echo "2. Check RViz - wheels should be visible"
echo "3. Drive robot - map should form"
echo ""
echo "If issues persist, see TF_CONFLICT_FIX.md"
echo "========================================"
