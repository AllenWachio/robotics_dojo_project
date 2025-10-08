#!/bin/bash

# NAVIGATION DIAGNOSTICS SCRIPT
# Run this after launching navigation to verify everything is working
# This will help identify issues before you try to send goals

echo "=========================================="
echo "  Navigation System Diagnostics"
echo "=========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Track overall status
all_checks_passed=true

# Function to check if a command is available
check_command() {
    if command -v $1 &> /dev/null; then
        return 0
    else
        return 1
    fi
}

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "1. CHECKING ROS2 NODE STATUS"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Check if ROS nodes are running
echo "Checking for Nav2 core nodes..."

required_nav_nodes=(
    "controller_server"
    "planner_server"
    "bt_navigator"
    "behavior_server"
)

nodes_output=$(ros2 node list 2>/dev/null)

for node in "${required_nav_nodes[@]}"; do
    if echo "$nodes_output" | grep -q "/$node"; then
        echo -e "   ${GREEN}✓${NC} $node is running"
    else
        echo -e "   ${RED}✗${NC} $node is NOT running"
        all_checks_passed=false
    fi
done

# Check for localization
echo ""
echo "Checking for localization node..."
if echo "$nodes_output" | grep -q "/amcl\|/slam_toolbox"; then
    if echo "$nodes_output" | grep -q "/amcl"; then
        echo -e "   ${GREEN}✓${NC} AMCL localization is running"
    else
        echo -e "   ${GREEN}✓${NC} SLAM Toolbox localization is running"
    fi
else
    echo -e "   ${RED}✗${NC} No localization node found (AMCL or SLAM Toolbox)"
    echo -e "   ${YELLOW}⚠${NC}  Without localization, navigation won't work!"
    all_checks_passed=false
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "2. CHECKING TF TRANSFORMS"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

echo "Checking critical TF transforms..."

# Check map -> odom transform
echo -n "   map -> odom: "
if timeout 2 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -q "At time"; then
    echo -e "${GREEN}✓${NC} Transform available"
else
    echo -e "${RED}✗${NC} Transform NOT available"
    echo -e "   ${YELLOW}⚠${NC}  This is critical! Localization node must provide this."
    all_checks_passed=false
fi

# Check odom -> base_link transform
echo -n "   odom -> base_link: "
if timeout 2 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -q "At time"; then
    echo -e "${GREEN}✓${NC} Transform available"
else
    echo -e "${RED}✗${NC} Transform NOT available"
    echo -e "   ${YELLOW}⚠${NC}  Odometry is not being published!"
    all_checks_passed=false
fi

# Check full chain map -> base_link
echo -n "   map -> base_link: "
if timeout 2 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -q "At time"; then
    echo -e "${GREEN}✓${NC} Full transform chain working"
else
    echo -e "${RED}✗${NC} Full chain broken"
    all_checks_passed=false
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "3. CHECKING CRITICAL TOPICS"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

echo "Checking topic availability..."

critical_topics=(
    "/goal_pose:Goal commands"
    "/cmd_vel:Velocity commands to robot"
    "/scan:LiDAR data"
    "/odom:Odometry"
    "/map:Map data"
    "/plan:Global path"
)

for topic_info in "${critical_topics[@]}"; do
    topic="${topic_info%%:*}"
    description="${topic_info##*:}"
    
    if ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
        echo -e "   ${GREEN}✓${NC} $topic ($description)"
    else
        echo -e "   ${RED}✗${NC} $topic ($description) - NOT FOUND"
        all_checks_passed=false
    fi
done

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "4. CHECKING TOPIC DATA FLOW"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

echo "Checking if topics are actively publishing..."

# Check /scan
echo -n "   /scan (LiDAR): "
if timeout 3 ros2 topic echo /scan --once &>/dev/null; then
    hz=$(timeout 3 ros2 topic hz /scan 2>&1 | grep "average rate" | awk '{print $3}')
    echo -e "${GREEN}✓${NC} Publishing (~${hz} Hz)"
else
    echo -e "${RED}✗${NC} No data"
    all_checks_passed=false
fi

# Check /odom
echo -n "   /odom (Odometry): "
if timeout 3 ros2 topic echo /odom --once &>/dev/null; then
    hz=$(timeout 3 ros2 topic hz /odom 2>&1 | grep "average rate" | awk '{print $3}')
    echo -e "${GREEN}✓${NC} Publishing (~${hz} Hz)"
else
    echo -e "${RED}✗${NC} No data"
    all_checks_passed=false
fi

# Check /map
echo -n "   /map (Map data): "
if timeout 3 ros2 topic echo /map --once &>/dev/null; then
    echo -e "${GREEN}✓${NC} Map available"
else
    echo -e "${RED}✗${NC} No map data"
    echo -e "   ${YELLOW}⚠${NC}  Navigation needs a map!"
    all_checks_passed=false
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "5. CHECKING LIFECYCLE STATES"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

echo "Checking Nav2 node lifecycle states..."

lifecycle_nodes=$(ros2 lifecycle nodes 2>/dev/null)

if [ -z "$lifecycle_nodes" ]; then
    echo -e "   ${YELLOW}⚠${NC}  No lifecycle nodes found (this might be okay)"
else
    echo "$lifecycle_nodes" | while read -r node; do
        if [ ! -z "$node" ]; then
            state=$(ros2 lifecycle get "$node" 2>/dev/null | grep "active\|inactive\|unconfigured" | head -1)
            if echo "$state" | grep -q "active"; then
                echo -e "   ${GREEN}✓${NC} $node: active"
            else
                echo -e "   ${RED}✗${NC} $node: $state"
                all_checks_passed=false
            fi
        fi
    done
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "6. CHECKING COSTMAPS"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

echo "Checking costmap topics..."

# Check global costmap
echo -n "   /global_costmap/costmap: "
if timeout 3 ros2 topic echo /global_costmap/costmap --once &>/dev/null; then
    echo -e "${GREEN}✓${NC} Publishing"
else
    echo -e "${RED}✗${NC} No data"
    all_checks_passed=false
fi

# Check local costmap
echo -n "   /local_costmap/costmap: "
if timeout 3 ros2 topic echo /local_costmap/costmap --once &>/dev/null; then
    echo -e "${GREEN}✓${NC} Publishing"
else
    echo -e "${RED}✗${NC} No data"
    all_checks_passed=false
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "7. CMD_VEL CONNECTIVITY TEST"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

echo "Testing /cmd_vel topic..."

# Check if cmd_vel topic exists
if ros2 topic list 2>/dev/null | grep -q "^/cmd_vel$"; then
    echo -e "   ${GREEN}✓${NC} /cmd_vel topic exists"
    
    # Check who's publishing to it
    publishers=$(ros2 topic info /cmd_vel 2>/dev/null | grep "Publisher count" | awk '{print $3}')
    subscribers=$(ros2 topic info /cmd_vel 2>/dev/null | grep "Subscription count" | awk '{print $3}')
    
    echo "   Publishers: $publishers"
    echo "   Subscribers: $subscribers"
    
    if [ "$subscribers" -eq "0" ]; then
        echo -e "   ${RED}✗${NC} No subscribers! Robot won't receive velocity commands!"
        all_checks_passed=false
    else
        echo -e "   ${GREEN}✓${NC} Robot is subscribed to velocity commands"
    fi
    
    # Test if we can publish to cmd_vel
    echo ""
    echo "   Testing manual velocity command..."
    echo "   (Publishing small forward velocity for 1 second)"
    
    timeout 1 ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" &>/dev/null
    
    if [ $? -eq 0 ]; then
        echo -e "   ${GREEN}✓${NC} Can publish to /cmd_vel"
    else
        echo -e "   ${RED}✗${NC} Cannot publish to /cmd_vel"
        all_checks_passed=false
    fi
else
    echo -e "   ${RED}✗${NC} /cmd_vel topic does NOT exist!"
    echo -e "   ${YELLOW}⚠${NC}  This is critical - navigation cannot control the robot!"
    all_checks_passed=false
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "8. CHECKING ROBOT LOCALIZATION"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

echo "Checking if robot is localized on the map..."

# Check if amcl is publishing pose
if ros2 topic list 2>/dev/null | grep -q "/particle_cloud"; then
    echo "   Checking AMCL particle cloud..."
    particle_count=$(timeout 3 ros2 topic echo /particle_cloud --once 2>/dev/null | grep -c "pose:")
    
    if [ "$particle_count" -gt 0 ]; then
        echo -e "   ${GREEN}✓${NC} AMCL has $particle_count particles"
        echo -e "   ${YELLOW}⚠${NC}  Remember to set initial pose with '2D Pose Estimate' in RViz!"
    else
        echo -e "   ${YELLOW}⚠${NC}  AMCL not yet localized. Set initial pose in RViz!"
    fi
elif ros2 topic list 2>/dev/null | grep -q "/slam_toolbox"; then
    echo -e "   ${GREEN}✓${NC} Using SLAM Toolbox localization"
    echo -e "   ${YELLOW}⚠${NC}  Make sure to set initial pose with '2D Pose Estimate' in RViz!"
else
    echo -e "   ${YELLOW}⚠${NC}  Cannot determine localization status"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "9. SYSTEM SUMMARY"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

if [ "$all_checks_passed" = true ]; then
    echo -e "${GREEN}✓✓✓ ALL CHECKS PASSED! ✓✓✓${NC}"
    echo ""
    echo "Your navigation system is ready!"
    echo ""
    echo "Next steps:"
    echo "1. Open RViz (should already be open)"
    echo "2. Click '2D Pose Estimate' tool"
    echo "3. Set robot's initial position on map"
    echo "4. Wait 3-5 seconds for localization to stabilize"
    echo "5. Click 'Nav2 Goal' tool"
    echo "6. Set destination on map"
    echo "7. Robot should navigate automatically!"
    echo ""
else
    echo -e "${RED}✗✗✗ ISSUES DETECTED! ✗✗✗${NC}"
    echo ""
    echo "Navigation system has problems that need fixing."
    echo "Review the errors above and fix them before sending goals."
    echo ""
    echo "Common fixes:"
    echo "- If Nav2 nodes missing: Did you run the navigation launch script?"
    echo "- If TF transforms missing: Check localization node is running"
    echo "- If /cmd_vel has no subscribers: Check robot hardware is running"
    echo "- If topics not publishing: Check sensors and hardware connections"
    echo ""
fi

echo "=========================================="
echo "  Diagnostics Complete"
echo "=========================================="
