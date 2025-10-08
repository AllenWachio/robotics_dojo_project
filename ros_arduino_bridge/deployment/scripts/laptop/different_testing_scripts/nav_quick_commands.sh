#!/bin/bash

###############################################################################
# NAV2 QUICK COMMANDS
# Useful shortcuts for common navigation tasks
#
# Usage: ./nav_quick_commands.sh [command]
###############################################################################

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

print_success() { echo -e "${GREEN}✓${NC} $1"; }
print_error() { echo -e "${RED}✗${NC} $1"; }
print_info() { echo -e "${CYAN}ℹ${NC} $1"; }
print_warning() { echo -e "${YELLOW}⚠${NC} $1"; }

# Command functions
set_initial_pose() {
    echo -e "${CYAN}Setting Initial Pose${NC}"
    echo "This will set the robot's initial pose on the map"
    echo ""
    read -p "X position (meters): " x
    read -p "Y position (meters): " y
    read -p "Yaw angle (radians, 0 = facing right): " yaw
    
    echo ""
    print_info "Publishing initial pose: x=$x, y=$y, yaw=$yaw"
    
    ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{
      header: {
        stamp: {sec: 0, nanosec: 0},
        frame_id: 'map'
      },
      pose: {
        pose: {
          position: {x: $x, y: $y, z: 0.0},
          orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
        },
        covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
      }
    }"
    
    print_success "Initial pose published!"
}

send_goal() {
    echo -e "${CYAN}Sending Navigation Goal${NC}"
    echo "This will send the robot to a goal position"
    echo ""
    read -p "Goal X position (meters): " x
    read -p "Goal Y position (meters): " y
    read -p "Goal Yaw angle (radians): " yaw
    
    echo ""
    print_info "Sending goal: x=$x, y=$y, yaw=$yaw"
    
    ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
      header: {
        stamp: {sec: 0, nanosec: 0},
        frame_id: 'map'
      },
      pose: {
        position: {x: $x, y: $y, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      }
    }"
    
    print_success "Goal sent!"
}

cancel_navigation() {
    echo -e "${CYAN}Canceling Navigation${NC}"
    ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose {} --feedback
    print_info "Navigation canceled"
}

clear_costmaps() {
    echo -e "${CYAN}Clearing Costmaps${NC}"
    print_info "Clearing global costmap..."
    ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap
    
    print_info "Clearing local costmap..."
    ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap
    
    print_success "Costmaps cleared!"
}

reinitialize_amcl() {
    echo -e "${CYAN}Reinitializing AMCL${NC}"
    print_warning "This will reset localization - you'll need to set initial pose again"
    read -p "Continue? (y/n): " -n 1 -r
    echo
    
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
        print_success "AMCL reinitialized - please set initial pose in RViz"
    fi
}

get_robot_pose() {
    echo -e "${CYAN}Current Robot Pose${NC}"
    print_info "Getting pose from AMCL..."
    ros2 topic echo /amcl_pose --once
}

check_goal_status() {
    echo -e "${CYAN}Navigation Goal Status${NC}"
    print_info "Checking navigation action status..."
    ros2 action list -t
    echo ""
    print_info "To see detailed status:"
    echo "ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose {} --feedback"
}

emergency_stop() {
    echo -e "${RED}EMERGENCY STOP${NC}"
    print_warning "Publishing zero velocity..."
    
    ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{
      linear: {x: 0.0, y: 0.0, z: 0.0},
      angular: {x: 0.0, y: 0.0, z: 0.0}
    }"
    
    print_success "Stop command sent!"
}

watch_navigation() {
    echo -e "${CYAN}Watching Navigation Topics${NC}"
    echo "Press Ctrl+C to stop"
    echo ""
    
    print_info "Position (AMCL):"
    ros2 topic echo /amcl_pose &
    PID1=$!
    
    print_info "Velocity Commands:"
    ros2 topic echo /cmd_vel &
    PID2=$!
    
    wait $PID1 $PID2
}

lifecycle_configure() {
    echo -e "${CYAN}Configuring Lifecycle Nodes${NC}"
    
    nodes=("map_server" "amcl" "controller_server" "planner_server" "behavior_server" "bt_navigator")
    
    for node in "${nodes[@]}"; do
        if ros2 node list 2>/dev/null | grep -q "/$node"; then
            print_info "Configuring $node..."
            ros2 lifecycle set /$node configure
            ros2 lifecycle set /$node activate
        fi
    done
    
    print_success "Lifecycle nodes configured and activated!"
}

show_map_info() {
    echo -e "${CYAN}Map Information${NC}"
    print_info "Map metadata:"
    ros2 topic echo /map_metadata --once
    
    echo ""
    print_info "Map origin and size will be shown above"
}

show_costmap_info() {
    echo -e "${CYAN}Costmap Information${NC}"
    
    echo "Global Costmap:"
    ros2 topic echo /global_costmap/costmap --once | head -20
    
    echo ""
    echo "Local Costmap:"
    ros2 topic echo /local_costmap/costmap --once | head -20
}

teleop_manual() {
    echo -e "${CYAN}Manual Teleoperation${NC}"
    print_info "Starting keyboard teleop..."
    print_warning "This will override autonomous navigation!"
    echo ""
    
    if command -v ros2 run teleop_twist_keyboard teleop_twist_keyboard &> /dev/null; then
        ros2 run teleop_twist_keyboard teleop_twist_keyboard
    else
        print_error "teleop_twist_keyboard not installed!"
        print_info "Install with: sudo apt install ros-humble-teleop-twist-keyboard"
    fi
}

show_help() {
    cat << EOF
${GREEN}╔════════════════════════════════════════════════════════════╗${NC}
${GREEN}║         NAV2 QUICK COMMANDS - NAVIGATION SHORTCUTS        ║${NC}
${GREEN}╚════════════════════════════════════════════════════════════╝${NC}

${YELLOW}Usage:${NC}
  ./nav_quick_commands.sh [command]

${YELLOW}Commands:${NC}

${CYAN}Localization:${NC}
  ${GREEN}init-pose${NC}        Set initial pose (localize robot on map)
  ${GREEN}reinit-amcl${NC}      Reinitialize AMCL (reset localization)
  ${GREEN}get-pose${NC}         Get current robot pose

${CYAN}Navigation:${NC}
  ${GREEN}goal${NC}             Send navigation goal
  ${GREEN}cancel${NC}           Cancel current navigation
  ${GREEN}goal-status${NC}      Check navigation goal status
  ${GREEN}watch${NC}            Watch navigation topics (pose + cmd_vel)

${CYAN}Costmaps:${NC}
  ${GREEN}clear-costmap${NC}    Clear both global and local costmaps
  ${GREEN}costmap-info${NC}     Show costmap information

${CYAN}Map:${NC}
  ${GREEN}map-info${NC}         Show map metadata

${CYAN}Control:${NC}
  ${GREEN}stop${NC}             Emergency stop (send zero velocity)
  ${GREEN}teleop${NC}           Manual keyboard control (overrides nav)

${CYAN}System:${NC}
  ${GREEN}lifecycle${NC}        Configure and activate lifecycle nodes
  ${GREEN}help${NC}             Show this help message

${YELLOW}Examples:${NC}
  ${CYAN}# Localize robot at origin facing right${NC}
  ./nav_quick_commands.sh init-pose
  # Enter: x=0, y=0, yaw=0

  ${CYAN}# Send robot to position (2, 3)${NC}
  ./nav_quick_commands.sh goal
  # Enter: x=2, y=3, yaw=0

  ${CYAN}# Emergency stop${NC}
  ./nav_quick_commands.sh stop

  ${CYAN}# Check where robot thinks it is${NC}
  ./nav_quick_commands.sh get-pose

${YELLOW}Yaw Angle Reference:${NC}
  0 rad      = Facing right (East)
  π/2 rad    = Facing up (North)
  π rad      = Facing left (West)
  -π/2 rad   = Facing down (South)

${YELLOW}Pro Tips:${NC}
  • Use RViz for easier goal setting (2D Pose Estimate / Nav2 Goal buttons)
  • Clear costmaps if robot gets stuck or sees phantom obstacles
  • Use 'watch' to monitor navigation in real-time
  • Use 'stop' for emergency situations

EOF
}

# Main script logic
case "${1:-help}" in
    init-pose|init)
        set_initial_pose
        ;;
    goal|send-goal)
        send_goal
        ;;
    cancel|cancel-goal)
        cancel_navigation
        ;;
    clear-costmap|clear)
        clear_costmaps
        ;;
    reinit-amcl|reinit)
        reinitialize_amcl
        ;;
    get-pose|pose)
        get_robot_pose
        ;;
    goal-status|status)
        check_goal_status
        ;;
    stop|estop)
        emergency_stop
        ;;
    watch)
        watch_navigation
        ;;
    lifecycle)
        lifecycle_configure
        ;;
    map-info|map)
        show_map_info
        ;;
    costmap-info|costmap)
        show_costmap_info
        ;;
    teleop|manual)
        teleop_manual
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        print_error "Unknown command: $1"
        echo ""
        show_help
        exit 1
        ;;
esac
