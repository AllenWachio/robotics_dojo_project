#!/bin/bash

# REAL-TIME NAVIGATION MONITOR
# Watch critical topics during navigation
# Run this in a separate terminal while navigating

echo "=========================================="
echo "  Real-Time Navigation Monitor"
echo "=========================================="
echo ""
echo "Monitoring critical navigation topics..."
echo "Press Ctrl+C to stop"
echo ""

# Create a temporary directory for output files
TMPDIR=$(mktemp -d)
trap "rm -rf $TMPDIR" EXIT

# Function to monitor a topic and show status
monitor_topic() {
    local topic=$1
    local name=$2
    local output_file="$TMPDIR/${name}.txt"
    
    while true; do
        if timeout 1 ros2 topic echo "$topic" --once &> "$output_file"; then
            echo "✓" > "$TMPDIR/${name}_status.txt"
        else
            echo "✗" > "$TMPDIR/${name}_status.txt"
        fi
        sleep 0.5
    done
}

# Start monitoring in background
monitor_topic "/cmd_vel" "cmd_vel" &
monitor_topic "/scan" "scan" &
monitor_topic "/odom" "odom" &
monitor_topic "/plan" "plan" &

# Main display loop
while true; do
    clear
    echo "=========================================="
    echo "  Real-Time Navigation Monitor"
    echo "=========================================="
    echo ""
    date
    echo ""
    
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "TOPIC STATUS"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    
    # Check cmd_vel
    if [ -f "$TMPDIR/cmd_vel_status.txt" ]; then
        status=$(cat "$TMPDIR/cmd_vel_status.txt")
        if [ "$status" = "✓" ]; then
            echo -e "  /cmd_vel (Velocity commands):    \033[0;32m✓ Publishing\033[0m"
            
            # Get latest velocity
            if [ -f "$TMPDIR/cmd_vel.txt" ]; then
                linear=$(grep -A 3 "linear:" "$TMPDIR/cmd_vel.txt" | grep "x:" | awk '{print $2}')
                angular=$(grep -A 3 "angular:" "$TMPDIR/cmd_vel.txt" | grep "z:" | awk '{print $2}')
                echo "     → Linear: ${linear:-0.0} m/s, Angular: ${angular:-0.0} rad/s"
            fi
        else
            echo -e "  /cmd_vel (Velocity commands):    \033[0;31m✗ No data\033[0m"
        fi
    fi
    
    # Check scan
    if [ -f "$TMPDIR/scan_status.txt" ]; then
        status=$(cat "$TMPDIR/scan_status.txt")
        if [ "$status" = "✓" ]; then
            echo -e "  /scan (LiDAR):                   \033[0;32m✓ Publishing\033[0m"
        else
            echo -e "  /scan (LiDAR):                   \033[0;31m✗ No data\033[0m"
        fi
    fi
    
    # Check odom
    if [ -f "$TMPDIR/odom_status.txt" ]; then
        status=$(cat "$TMPDIR/odom_status.txt")
        if [ "$status" = "✓" ]; then
            echo -e "  /odom (Odometry):                \033[0;32m✓ Publishing\033[0m"
            
            # Get latest position
            if [ -f "$TMPDIR/odom.txt" ]; then
                pos_x=$(grep -A 3 "position:" "$TMPDIR/odom.txt" | grep "x:" | awk '{print $2}')
                pos_y=$(grep -A 3 "position:" "$TMPDIR/odom.txt" | grep "y:" | awk '{print $2}')
                echo "     → Position: x=${pos_x:-0.0}, y=${pos_y:-0.0}"
            fi
        else
            echo -e "  /odom (Odometry):                \033[0;31m✗ No data\033[0m"
        fi
    fi
    
    # Check plan
    if [ -f "$TMPDIR/plan_status.txt" ]; then
        status=$(cat "$TMPDIR/plan_status.txt")
        if [ "$status" = "✓" ]; then
            echo -e "  /plan (Global path):             \033[0;32m✓ Path exists\033[0m"
        else
            echo -e "  /plan (Global path):             \033[0;33m⚠ No path\033[0m"
        fi
    fi
    
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "NODE STATUS"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    
    # Check critical nodes
    nodes=$(ros2 node list 2>/dev/null)
    
    if echo "$nodes" | grep -q "/controller_server"; then
        echo -e "  Controller Server:               \033[0;32m✓ Running\033[0m"
    else
        echo -e "  Controller Server:               \033[0;31m✗ Not running\033[0m"
    fi
    
    if echo "$nodes" | grep -q "/planner_server"; then
        echo -e "  Planner Server:                  \033[0;32m✓ Running\033[0m"
    else
        echo -e "  Planner Server:                  \033[0;31m✗ Not running\033[0m"
    fi
    
    if echo "$nodes" | grep -q "/bt_navigator"; then
        echo -e "  BT Navigator:                    \033[0;32m✓ Running\033[0m"
    else
        echo -e "  BT Navigator:                    \033[0;31m✗ Not running\033[0m"
    fi
    
    if echo "$nodes" | grep -q "/amcl\|/slam_toolbox"; then
        echo -e "  Localization:                    \033[0;32m✓ Running\033[0m"
    else
        echo -e "  Localization:                    \033[0;31m✗ Not running\033[0m"
    fi
    
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "NAVIGATION STATUS"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    
    # Determine navigation status
    if [ -f "$TMPDIR/cmd_vel_status.txt" ] && [ "$(cat $TMPDIR/cmd_vel_status.txt)" = "✓" ]; then
        if [ -f "$TMPDIR/cmd_vel.txt" ]; then
            linear=$(grep -A 3 "linear:" "$TMPDIR/cmd_vel.txt" | grep "x:" | awk '{print $2}')
            angular=$(grep -A 3 "angular:" "$TMPDIR/cmd_vel.txt" | grep "z:" | awk '{print $2}')
            
            # Check if moving
            if [ "$linear" != "0.0" ] || [ "$angular" != "0.0" ]; then
                echo -e "  Status: \033[0;32m🚀 NAVIGATING\033[0m"
            else
                echo -e "  Status: \033[0;33m⏸️  IDLE (at goal or waiting)\033[0m"
            fi
        fi
    else
        echo -e "  Status: \033[0;31m❌ NOT NAVIGATING\033[0m"
        echo ""
        echo "  Possible reasons:"
        echo "  - No goal has been set"
        echo "  - Navigation stack not running"
        echo "  - Robot reached goal"
    fi
    
    echo ""
    echo "Press Ctrl+C to stop monitoring"
    echo ""
    
    sleep 1
done
