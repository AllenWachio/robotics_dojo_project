#!/bin/bash

###############################################################################
# NAV2 MONITORING SCRIPT
# Quick diagnostics and health checks for your navigation system
#
# Usage: ./navigation_monitor.sh [option]
# Options:
#   status    - Show all node states
#   topics    - Show all active topics
#   tf        - Check TF tree
#   localize  - Check localization status
#   nav       - Check navigation status
#   full      - Run all diagnostics
###############################################################################

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

print_header() {
    echo -e "\n${BLUE}═══════════════════════════════════════════════════════════${NC}"
    echo -e "${CYAN}$1${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}\n"
}

print_success() {
    echo -e "${GREEN}✓${NC} $1"
}

print_error() {
    echo -e "${RED}✗${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} $1"
}

print_info() {
    echo -e "${CYAN}ℹ${NC} $1"
}

check_node_exists() {
    if ros2 node list 2>/dev/null | grep -q "^$1$"; then
        print_success "Node running: $1"
        return 0
    else
        print_error "Node NOT running: $1"
        return 1
    fi
}

check_topic_exists() {
    if ros2 topic list 2>/dev/null | grep -q "^$1$"; then
        print_success "Topic active: $1"
        return 0
    else
        print_error "Topic NOT active: $1"
        return 1
    fi
}

check_status() {
    print_header "NAV2 NODE STATUS"
    
    echo "Localization Nodes:"
    check_node_exists "/map_server"
    check_node_exists "/amcl"
    
    echo -e "\nNavigation Nodes:"
    check_node_exists "/controller_server"
    check_node_exists "/planner_server"
    check_node_exists "/behavior_server"
    check_node_exists "/bt_navigator"
    check_node_exists "/waypoint_follower"
    check_node_exists "/velocity_smoother"
    
    echo -e "\nLifecycle Managers:"
    check_node_exists "/lifecycle_manager_localization" || check_node_exists "/lifecycle_manager_navigation"
    
    print_header "LIFECYCLE STATES"
    echo "Checking lifecycle node states..."
    ros2 lifecycle list 2>/dev/null || print_warning "No lifecycle nodes found"
}

check_topics() {
    print_header "CRITICAL TOPICS STATUS"
    
    echo "Sensor Topics:"
    check_topic_exists "/scan"
    check_topic_exists "/odom"
    
    echo -e "\nNavigation Topics:"
    check_topic_exists "/map"
    check_topic_exists "/plan"
    check_topic_exists "/cmd_vel"
    check_topic_exists "/amcl_pose"
    
    echo -e "\nCostmap Topics:"
    check_topic_exists "/local_costmap/costmap"
    check_topic_exists "/global_costmap/costmap"
    
    print_header "TOPIC FREQUENCIES"
    echo "Checking critical topic rates (3 second samples)..."
    echo ""
    
    echo -n "Scan rate: "
    ros2 topic hz /scan --window 3 2>/dev/null | grep "average rate" || print_error "No data on /scan"
    
    echo -n "Odom rate: "
    ros2 topic hz /odom --window 3 2>/dev/null | grep "average rate" || print_error "No data on /odom"
    
    echo -n "cmd_vel rate: "
    ros2 topic hz /cmd_vel --window 3 2>/dev/null | grep "average rate" || print_warning "No data on /cmd_vel (normal if not navigating)"
}

check_tf() {
    print_header "TF TREE CHECK"
    
    echo "Checking critical transforms..."
    
    # Check if TF topics exist
    if ! check_topic_exists "/tf"; then
        print_error "TF topic not active!"
        return 1
    fi
    
    # Try to get specific transforms
    echo -e "\nChecking transform: base_link → laser"
    timeout 2 ros2 run tf2_ros tf2_echo base_link laser 2>/dev/null && print_success "Transform OK" || print_error "Transform failed"
    
    echo -e "\nChecking transform: odom → base_link"
    timeout 2 ros2 run tf2_ros tf2_echo odom base_link 2>/dev/null && print_success "Transform OK" || print_error "Transform failed"
    
    echo -e "\nChecking transform: map → base_link (requires AMCL)"
    timeout 2 ros2 run tf2_ros tf2_echo map base_link 2>/dev/null && print_success "Transform OK" || print_warning "Transform failed (normal if not localized)"
    
    print_info "Generating full TF tree diagram..."
    ros2 run tf2_tools view_frames 2>/dev/null
    if [ -f "frames.pdf" ]; then
        print_success "TF tree saved to frames.pdf"
        print_info "Open with: xdg-open frames.pdf"
    fi
}

check_localization() {
    print_header "LOCALIZATION STATUS"
    
    echo "Checking AMCL localization..."
    
    if ! check_node_exists "/amcl"; then
        return 1
    fi
    
    echo -e "\nChecking map availability:"
    if timeout 2 ros2 topic echo /map --once >/dev/null 2>&1; then
        print_success "Map is being published"
        
        # Get map info
        echo -e "\nMap metadata:"
        timeout 2 ros2 topic echo /map_metadata --once 2>/dev/null | head -20
    else
        print_error "Map not available"
    fi
    
    echo -e "\nChecking robot pose estimate:"
    if timeout 2 ros2 topic echo /amcl_pose --once >/dev/null 2>&1; then
        print_success "AMCL publishing pose"
        echo -e "\nCurrent pose estimate:"
        timeout 2 ros2 topic echo /amcl_pose --once 2>/dev/null | head -15
    else
        print_warning "AMCL not publishing pose (set initial pose in RViz)"
    fi
    
    echo -e "\nParticle cloud status:"
    if timeout 2 ros2 topic echo /particlecloud --once >/dev/null 2>&1; then
        PARTICLE_COUNT=$(timeout 2 ros2 topic echo /particlecloud --once 2>/dev/null | grep -c "position:")
        print_success "Particle cloud active with ~$PARTICLE_COUNT particles"
    else
        print_warning "Particle cloud not available"
    fi
}

check_navigation() {
    print_header "NAVIGATION STATUS"
    
    echo "Navigation Servers:"
    check_node_exists "/controller_server"
    check_node_exists "/planner_server"
    check_node_exists "/bt_navigator"
    
    echo -e "\nChecking planned path:"
    if timeout 2 ros2 topic echo /plan --once >/dev/null 2>&1; then
        print_success "Path planner is active"
        WAYPOINTS=$(timeout 2 ros2 topic echo /plan --once 2>/dev/null | grep -c "pose:")
        print_info "Current plan has $WAYPOINTS waypoints"
    else
        print_warning "No planned path (set goal in RViz)"
    fi
    
    echo -e "\nChecking velocity commands:"
    if timeout 2 ros2 topic echo /cmd_vel --once >/dev/null 2>&1; then
        print_success "Velocity commands being published"
        echo -e "\nCurrent cmd_vel:"
        timeout 2 ros2 topic echo /cmd_vel --once 2>/dev/null
    else
        print_warning "No velocity commands (normal if not navigating)"
    fi
    
    echo -e "\nCostmap status:"
    check_topic_exists "/local_costmap/costmap"
    check_topic_exists "/global_costmap/costmap"
}

run_full_diagnostics() {
    echo -e "${GREEN}╔════════════════════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║         NAV2 FULL DIAGNOSTICS REPORT                       ║${NC}"
    echo -e "${GREEN}╚════════════════════════════════════════════════════════════╝${NC}"
    
    check_status
    check_topics
    check_tf
    check_localization
    check_navigation
    
    print_header "DIAGNOSTICS COMPLETE"
    print_info "Review the results above to identify any issues"
}

show_help() {
    cat << EOF
${CYAN}NAV2 Navigation Monitor${NC}

${YELLOW}Usage:${NC}
  ./navigation_monitor.sh [option]

${YELLOW}Options:${NC}
  ${GREEN}status${NC}      Show all node states and lifecycle status
  ${GREEN}topics${NC}      Show active topics and their frequencies
  ${GREEN}tf${NC}          Check TF tree and critical transforms
  ${GREEN}localize${NC}    Check localization (AMCL) status
  ${GREEN}nav${NC}         Check navigation server status
  ${GREEN}full${NC}        Run all diagnostics (recommended)
  ${GREEN}help${NC}        Show this help message

${YELLOW}Examples:${NC}
  ./navigation_monitor.sh full       # Run complete diagnostics
  ./navigation_monitor.sh status     # Quick node status check
  ./navigation_monitor.sh localize   # Check if robot is localized

${YELLOW}Quick Checks:${NC}
  ${CYAN}# Are all nodes running?${NC}
  ros2 node list

  ${CYAN}# Is robot localized?${NC}
  ros2 topic echo /amcl_pose --once

  ${CYAN}# Is map loaded?${NC}
  ros2 topic echo /map --once

  ${CYAN}# Are velocity commands being sent?${NC}
  ros2 topic hz /cmd_vel

EOF
}

# Main script logic
case "${1:-full}" in
    status)
        check_status
        ;;
    topics)
        check_topics
        ;;
    tf)
        check_tf
        ;;
    localize)
        check_localization
        ;;
    nav)
        check_navigation
        ;;
    full)
        run_full_diagnostics
        ;;
    help|--help|-h)
        show_help
        ;;
    *)
        print_error "Unknown option: $1"
        echo ""
        show_help
        exit 1
        ;;
esac
