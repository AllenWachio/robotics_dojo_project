#!/bin/bash
################################################################################
# 06 - Test Incremental Navigation with Obstacle Avoidance
# 
# This script tests the new IncrementalMove behavior with:
# - Waypoint generation (0.2m steps)
# - Progress monitoring
# - LiDAR-based obstacle avoidance
# - Automatic recovery from stuck situations
# - Detailed movement logging
#
# Prerequisites:
# - Navigation stack running (SLAM + Nav2)
# - AMCL localization active
# - LiDAR publishing to /scan
# - Robot properly localized on map
#
# Author: Robotics Dojo 2025
# Date: October 6, 2025
################################################################################

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}============================================================${NC}"
echo -e "${BLUE}  TEST: Incremental Navigation with Obstacle Avoidance${NC}"
echo -e "${BLUE}============================================================${NC}"

# Function to check if a topic exists
check_topic() {
    local topic=$1
    local name=$2
    echo -n "Checking $name... "
    if ros2 topic info $topic &> /dev/null; then
        echo -e "${GREEN}✓ Active${NC}"
        return 0
    else
        echo -e "${RED}✗ Not found${NC}"
        return 1
    fi
}

# Function to check if a node is running
check_node() {
    local node=$1
    local name=$2
    echo -n "Checking $name... "
    if ros2 node info $node &> /dev/null; then
        echo -e "${GREEN}✓ Running${NC}"
        return 0
    else
        echo -e "${RED}✗ Not running${NC}"
        return 1
    fi
}

# Function to check if an action server is available
check_action() {
    local action=$1
    local name=$2
    echo -n "Checking $name... "
    if ros2 action list | grep -q $action; then
        echo -e "${GREEN}✓ Available${NC}"
        return 0
    else
        echo -e "${RED}✗ Not available${NC}"
        return 1
    fi
}

################################################################################
# PRE-FLIGHT CHECKS
################################################################################

echo ""
echo -e "${YELLOW}[1/4] Pre-Flight System Checks${NC}"
echo "--------------------------------------------------------------"

all_checks_passed=true

# Check Navigation Stack
echo -e "\n${BLUE}Navigation Stack:${NC}"
check_action "/navigate_to_pose" "Nav2 Action Server" || all_checks_passed=false
check_node "/controller_server" "Controller Server" || all_checks_passed=false
check_node "/planner_server" "Planner Server" || all_checks_passed=false

# Check Localization
echo -e "\n${BLUE}Localization:${NC}"
check_topic "/amcl_pose" "AMCL Pose" || all_checks_passed=false
check_node "/amcl" "AMCL Node" || all_checks_passed=false

# Check Sensors
echo -e "\n${BLUE}Sensors:${NC}"
check_topic "/scan" "LiDAR Scan" || all_checks_passed=false

# Check Map
echo -e "\n${BLUE}Map:${NC}"
check_topic "/map" "Map Topic" || all_checks_passed=false

# Summary
echo ""
echo "--------------------------------------------------------------"
if [ "$all_checks_passed" = true ]; then
    echo -e "${GREEN}✓ All pre-flight checks passed!${NC}"
else
    echo -e "${RED}✗ Some checks failed!${NC}"
    echo ""
    echo -e "${YELLOW}Required services not available:${NC}"
    echo "  Make sure navigation stack is running:"
    echo "    ./03_navigation.sh"
    echo ""
    echo "  Or SLAM + localization:"
    echo "    ./full_slam_test.launch.py"
    echo ""
    read -p "Continue anyway? (y/N): " continue_choice
    if [[ ! $continue_choice =~ ^[Yy]$ ]]; then
        echo -e "${RED}Aborting.${NC}"
        exit 1
    fi
fi

################################################################################
# INFORMATION
################################################################################

echo ""
echo -e "${YELLOW}[2/4] Mission Information${NC}"
echo "--------------------------------------------------------------"
echo -e "${BLUE}Test Mission Features:${NC}"
echo "  • Incremental movement with 0.2m steps"
echo "  • Real-time position tracking and logging"
echo "  • Obstacle detection using LiDAR"
echo "  • Automatic recovery and alternative routing"
echo "  • Progress monitoring at each waypoint"
echo "  • Movement statistics and analysis"
echo ""
echo -e "${BLUE}Mission Plan:${NC}"
echo "  1. Wait for mapping (simulated)"
echo "  2. Move to Position A (0.5, 0.5)"
echo "  3. Perform task at Position A"
echo "  4. Move to Position B (0.5, -0.5)"
echo "  5. Perform task at Position B"
echo "  6. Return to start (0.0, 0.0)"
echo "  7. Complete mission"
echo ""
echo -e "${YELLOW}NOTE:${NC} These are safe test coordinates."
echo "      Modify test_incremental_navigation.py for your specific waypoints."
echo ""

################################################################################
# LOCALIZATION CHECK
################################################################################

echo -e "${YELLOW}[3/4] Verifying Robot Localization${NC}"
echo "--------------------------------------------------------------"
echo "Checking robot position from AMCL..."

# Get current pose
current_pose=$(ros2 topic echo /amcl_pose --once 2>/dev/null | grep -A 3 "position:" | tail -3)

if [ -n "$current_pose" ]; then
    echo -e "${GREEN}✓ Robot is localized!${NC}"
    echo "$current_pose"
    echo ""
    echo -e "${YELLOW}IMPORTANT:${NC} If this position looks wrong, re-localize robot in RViz:"
    echo "  1. Use '2D Pose Estimate' tool"
    echo "  2. Click robot's actual position on map"
    echo "  3. Drag to set orientation"
else
    echo -e "${RED}✗ Could not get robot position!${NC}"
    echo ""
    echo -e "${YELLOW}Please localize robot in RViz before starting mission.${NC}"
    read -p "Continue anyway? (y/N): " continue_choice
    if [[ ! $continue_choice =~ ^[Yy]$ ]]; then
        echo -e "${RED}Aborting.${NC}"
        exit 1
    fi
fi

################################################################################
# LAUNCH
################################################################################

echo ""
echo -e "${YELLOW}[4/4] Launching Test Mission${NC}"
echo "--------------------------------------------------------------"
echo -e "${GREEN}Starting incremental navigation behavior tree...${NC}"
echo ""
echo -e "${YELLOW}Watch for:${NC}"
echo "  • Waypoint progress logs"
echo "  • Distance traveled at each step"
echo "  • Obstacle detection messages"
echo "  • Recovery attempts (if stuck)"
echo "  • Final movement statistics"
echo ""
echo -e "${BLUE}============================================================${NC}"
echo ""

# Add behavior_tree directory to Python path
export PYTHONPATH="${PYTHONPATH}:${HOME}/ros2_ws/src/ros_arduino_bridge/behavior_tree"

# Launch the behavior tree directly (simpler than launch file for testing)
cd ~/ros2_ws/src/ros_arduino_bridge/behavior_tree

# Run the test
python3 test_incremental_navigation.py

################################################################################
# POST-MISSION
################################################################################

echo ""
echo -e "${BLUE}============================================================${NC}"
echo -e "${BLUE}  Mission Complete${NC}"
echo -e "${BLUE}============================================================${NC}"
echo ""
echo -e "${GREEN}Next Steps:${NC}"
echo "  • Review movement statistics in the output above"
echo "  • Check waypoint history and path efficiency"
echo "  • Modify waypoints in test_incremental_navigation.py for your needs"
echo "  • Use IncrementalMove in your own missions!"
echo ""
echo -e "${YELLOW}Integration:${NC}"
echo "  IncrementalMove coexists with MoveToPosition."
echo "  Use IncrementalMove when you need:"
echo "    - Progress monitoring"
echo "    - Obstacle avoidance"
echo "    - Detailed logging"
echo "  Use MoveToPosition for simple direct movements."
echo ""
echo -e "${BLUE}============================================================${NC}"
