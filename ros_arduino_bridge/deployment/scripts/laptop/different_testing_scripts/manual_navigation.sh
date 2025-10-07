#!/bin/bash

###############################################################################
# AUTOMATED NAV2 LAUNCH SCRIPT FOR UBUNTU 22.04
# This script launches all Nav2 components in separate tmux panes automatically
# 
# Usage: ./manual_navigation.sh [map_name]
# Example: ./manual_navigation.sh gamefield
# Default map: gamefield
#
# Controls:
#   - Ctrl+b then arrow keys: Navigate between panes
#   - Ctrl+b then z: Zoom into current pane (toggle)
#   - Ctrl+b then [: Scroll mode (q to exit)
#   - Ctrl+b then d: Detach from session
#   - tmux attach -t nav2: Reattach to session
###############################################################################

# Configuration
SESSION_NAME="nav2"
MAP_NAME="${1:-gamefield}"
MAPS_DIR="$HOME/ros2_ws/maps"
CONFIG_DIR="$HOME/ros2_ws/src/ros_arduino_bridge/deployment/laptop/config"
MAP_FILE="$MAPS_DIR/${MAP_NAME}.yaml"
NAV2_PARAMS="$CONFIG_DIR/nav2_params.yaml"
RVIZ_CONFIG="$CONFIG_DIR/navigation_rviz_config.rviz"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Helper functions
print_header() {
    echo -e "${BLUE}================================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}================================================${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

# Check if tmux is installed
if ! command -v tmux &> /dev/null; then
    print_error "tmux is not installed!"
    echo "Install it with: sudo apt install tmux"
    exit 1
fi

# Check if map file exists
if [ ! -f "$MAP_FILE" ]; then
    print_error "Map file not found: $MAP_FILE"
    echo "Available maps in $MAPS_DIR:"
    ls -1 "$MAPS_DIR"/*.yaml 2>/dev/null | xargs -n 1 basename -s .yaml
    exit 1
fi

# Check if nav2_params exists
if [ ! -f "$NAV2_PARAMS" ]; then
    print_error "Nav2 params file not found: $NAV2_PARAMS"
    exit 1
fi

print_header "Automated Nav2 Launch Script"
echo "Map: $MAP_NAME"
echo "Map file: $MAP_FILE"
echo "Session: $SESSION_NAME"
echo ""

# Kill existing session if it exists
if tmux has-session -t $SESSION_NAME 2>/dev/null; then
    print_warning "Existing session found. Killing it..."
    tmux kill-session -t $SESSION_NAME
    sleep 1
fi

print_success "Creating new tmux session: $SESSION_NAME"

# Create new tmux session
tmux new-session -d -s $SESSION_NAME -n "Navigation"

# Source ROS2 setup in each pane
SOURCE_CMD="source $HOME/ros2_ws/install/setup.bash"

###############################################################################
# PANE LAYOUT:
# +------------------+------------------+
# |   Map Server     | AMCL             |
# +------------------+------------------+
# | Controller       | Planner          |
# +------------------+------------------+
# | Behavior         | BT Navigator     |
# +------------------+------------------+
# | Waypoint         | Velocity Smooth  |
# +------------------+------------------+
# | Lifecycle (Loc)  | Lifecycle (Nav)  |
# +------------------+------------------+
# | RViz             | Monitor          |
# +------------------+------------------+
###############################################################################

print_info "Setting up pane layout..."

# Split into 6 rows and 2 columns (12 panes total)
tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 2
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 4
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v
tmux select-pane -t 6
tmux split-window -v
tmux select-pane -t 2
tmux split-window -v
tmux select-pane -t 8
tmux split-window -v
tmux select-pane -t 4
tmux split-window -v

# Pane 0: Map Server
print_info "Configuring Map Server pane..."
tmux select-pane -t 0
tmux send-keys "$SOURCE_CMD" C-m
tmux send-keys "clear" C-m
tmux send-keys "echo '═══════════════════════════════════════'" C-m
tmux send-keys "echo '  MAP SERVER'" C-m
tmux send-keys "echo '═══════════════════════════════════════'" C-m
tmux send-keys "echo 'Map: $MAP_FILE'" C-m
tmux send-keys "echo 'Starting in 2 seconds...'" C-m
tmux send-keys "sleep 2" C-m
tmux send-keys "ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$MAP_FILE -p use_sim_time:=false" C-m

# Pane 1: AMCL
print_info "Configuring AMCL pane..."
tmux select-pane -t 1
tmux send-keys "$SOURCE_CMD" C-m
tmux send-keys "clear" C-m
tmux send-keys "echo '═══════════════════════════════════════'" C-m
tmux send-keys "echo '  AMCL - LOCALIZATION'" C-m
tmux send-keys "echo '═══════════════════════════════════════'" C-m
tmux send-keys "echo 'Waiting for Map Server...'" C-m
tmux send-keys "sleep 3" C-m
tmux send-keys "ros2 run nav2_amcl amcl --ros-args --params-file $NAV2_PARAMS -p use_sim_time:=false" C-m

# Pane 2: Lifecycle Manager (Localization)
print_info "Configuring Lifecycle Manager (Localization)..."
tmux select-pane -t 2
tmux send-keys "$SOURCE_CMD" C-m
tmux send-keys "clear" C-m
tmux send-keys "echo '═══════════════════════════════════════'" C-m
tmux send-keys "echo '  LIFECYCLE MANAGER - LOCALIZATION'" C-m
tmux send-keys "echo '═══════════════════════════════════════'" C-m
tmux send-keys "echo 'Waiting for Map Server and AMCL...'" C-m
tmux send-keys "sleep 5" C-m
tmux send-keys "ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p node_names:=\"['map_server','amcl']\" -p autostart:=true -p use_sim_time:=false" C-m

# Pane 3: Controller Server
print_info "Configuring Controller Server..."
tmux select-pane -t 3
tmux send-keys "$SOURCE_CMD" C-m
tmux send-keys "clear" C-m
tmux send-keys "echo '═══════════════════════════════════════'" C-m
tmux send-keys "echo '  CONTROLLER SERVER'" C-m
tmux send-keys "echo '═══════════════════════════════════════'" C-m
tmux send-keys "echo 'Waiting for localization...'" C-m
tmux send-keys "sleep 8" C-m
tmux send-keys "ros2 run nav2_controller controller_server --ros-args --params-file $NAV2_PARAMS -p use_sim_time:=false" C-m

# Pane 4: Planner Server
print_info "Configuring Planner Server..."
tmux select-pane -t 4
tmux send-keys "$SOURCE_CMD" C-m
tmux send-keys "clear" C-m
tmux send-keys "echo '═══════════════════════════════════════'" C-m
tmux send-keys "echo '  PLANNER SERVER'" C-m
tmux send-keys "echo '═══════════════════════════════════════'" C-m
tmux send-keys "echo 'Waiting for localization...'" C-m
tmux send-keys "sleep 8" C-m
tmux send-keys "ros2 run nav2_planner planner_server --ros-args --params-file $NAV2_PARAMS -p use_sim_time:=false" C-m

# Pane 5: Behavior Server
print_info "Configuring Behavior Server..."
tmux select-pane -t 5
tmux send-keys "$SOURCE_CMD" C-m
tmux send-keys "clear" C-m
tmux send-keys "echo '═══════════════════════════════════════'" C-m
tmux send-keys "echo '  BEHAVIOR SERVER'" C-m
tmux send-keys "echo '═══════════════════════════════════════'" C-m
tmux send-keys "echo 'Waiting for localization...'" C-m
tmux send-keys "sleep 8" C-m
tmux send-keys "ros2 run nav2_behaviors behavior_server --ros-args --params-file $NAV2_PARAMS -p use_sim_time:=false" C-m

# Pane 6: BT Navigator
print_info "Configuring BT Navigator..."
tmux select-pane -t 6
tmux send-keys "$SOURCE_CMD" C-m
tmux send-keys "clear" C-m
tmux send-keys "echo '═══════════════════════════════════════'" C-m
tmux send-keys "echo '  BT NAVIGATOR'" C-m
tmux send-keys "echo '═══════════════════════════════════════'" C-m
tmux send-keys "echo 'Waiting for other servers...'" C-m
tmux send-keys "sleep 10" C-m
tmux send-keys "ros2 run nav2_bt_navigator bt_navigator --ros-args --params-file $NAV2_PARAMS -p use_sim_time:=false" C-m

# Pane 7: Waypoint Follower
print_info "Configuring Waypoint Follower..."
tmux select-pane -t 7
tmux send-keys "$SOURCE_CMD" C-m
tmux send-keys "clear" C-m
tmux send-keys "echo '═══════════════════════════════════════'" C-m
tmux send-keys "echo '  WAYPOINT FOLLOWER'" C-m
tmux send-keys "echo '═══════════════════════════════════════'" C-m
tmux send-keys "echo 'Waiting for other servers...'" C-m
tmux send-keys "sleep 10" C-m
tmux send-keys "ros2 run nav2_waypoint_follower waypoint_follower --ros-args --params-file $NAV2_PARAMS -p use_sim_time:=false" C-m

# Pane 8: Velocity Smoother
print_info "Configuring Velocity Smoother..."
tmux select-pane -t 8
tmux send-keys "$SOURCE_CMD" C-m
tmux send-keys "clear" C-m
tmux send-keys "echo '═══════════════════════════════════════'" C-m
tmux send-keys "echo '  VELOCITY SMOOTHER'" C-m
tmux send-keys "echo '═══════════════════════════════════════'" C-m
tmux send-keys "echo 'Waiting for other servers...'" C-m
tmux send-keys "sleep 10" C-m
tmux send-keys "ros2 run nav2_velocity_smoother velocity_smoother --ros-args --params-file $NAV2_PARAMS -p use_sim_time:=false" C-m

# Pane 9: Lifecycle Manager (Navigation)
print_info "Configuring Lifecycle Manager (Navigation)..."
tmux select-pane -t 9
tmux send-keys "$SOURCE_CMD" C-m
tmux send-keys "clear" C-m
tmux send-keys "echo '═══════════════════════════════════════'" C-m
tmux send-keys "echo '  LIFECYCLE MANAGER - NAVIGATION'" C-m
tmux send-keys "echo '═══════════════════════════════════════'" C-m
tmux send-keys "echo 'Waiting for all navigation servers...'" C-m
tmux send-keys "sleep 12" C-m
tmux send-keys "ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args -p node_names:=\"['controller_server','planner_server','behavior_server','bt_navigator','waypoint_follower','velocity_smoother']\" -p autostart:=true -p use_sim_time:=false" C-m

# Pane 10: RViz
print_info "Configuring RViz..."
tmux select-pane -t 10
tmux send-keys "$SOURCE_CMD" C-m
tmux send-keys "clear" C-m
tmux send-keys "echo '═══════════════════════════════════════'" C-m
tmux send-keys "echo '  RVIZ2 - NAVIGATION INTERFACE'" C-m
tmux send-keys "echo '═══════════════════════════════════════'" C-m
tmux send-keys "echo 'Waiting for all nodes to start...'" C-m
tmux send-keys "sleep 15" C-m
if [ -f "$RVIZ_CONFIG" ]; then
    tmux send-keys "rviz2 -d $RVIZ_CONFIG" C-m
else
    print_warning "RViz config not found, launching with default config"
    tmux send-keys "rviz2" C-m
fi

# Pane 11: Monitoring/Commands
print_info "Configuring Monitor pane..."
tmux select-pane -t 11
tmux send-keys "$SOURCE_CMD" C-m
tmux send-keys "clear" C-m
tmux send-keys "cat << 'EOF'
╔════════════════════════════════════════════════════════════════════════╗
║         NAV2 AUTOMATED LAUNCH - MONITORING PANE                        ║
╚════════════════════════════════════════════════════════════════════════╝

🚀 AUTOMATIC LAUNCH SEQUENCE:
   ✓ Map Server          (starting at T+2s)
   ✓ AMCL                (starting at T+3s)
   ✓ Lifecycle Manager   (starting at T+5s)
   ✓ Controller Server   (starting at T+8s)
   ✓ Planner Server      (starting at T+8s)
   ✓ Behavior Server     (starting at T+8s)
   ✓ BT Navigator        (starting at T+10s)
   ✓ Waypoint Follower   (starting at T+10s)
   ✓ Velocity Smoother   (starting at T+10s)
   ✓ Lifecycle Manager   (starting at T+12s)
   ✓ RViz                (starting at T+15s)

📊 All components will start automatically!

⏱️  ESTIMATED READY TIME: ~20 seconds

🔍 USEFUL MONITORING COMMANDS:

# Check all running nodes
ros2 node list

# Check lifecycle states
ros2 lifecycle list

# Check topics
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /odom

# Check transforms
ros2 run tf2_ros tf2_echo map base_link

# Monitor navigation
ros2 topic echo /amcl_pose

🎮 NEXT STEPS IN RVIZ (after ~20 seconds):
   
   1. SET INITIAL POSE:
      - Click '2D Pose Estimate' button
      - Click where robot is on map
      - Drag to set orientation
      - Wait for particle cloud to converge
   
   2. SEND NAVIGATION GOAL:
      - Click 'Nav2 Goal' button
      - Click destination on map
      - Drag to set goal orientation
      - Watch robot navigate!

⌨️  TMUX CONTROLS:
   Ctrl+b → arrow keys  - Navigate between panes
   Ctrl+b → z           - Zoom current pane
   Ctrl+b → [           - Scroll mode (q to exit)
   Ctrl+b → d           - Detach session

🔧 QUICK ACTIONS (in new terminal):
   ./navigation_monitor.sh full      # Run diagnostics
   ./nav_quick_commands.sh stop      # Emergency stop
   ./nav_quick_commands.sh get-pose  # Check robot pose

📊 Map: $MAP_NAME
📂 Config: $NAV2_PARAMS

═══════════════════════════════════════════════════════════════════════

Waiting for system to initialize...

EOF
" C-m

# Add a countdown and status check
tmux send-keys "sleep 20" C-m
tmux send-keys "echo ''" C-m
tmux send-keys "echo '✓ System should be ready!'" C-m
tmux send-keys "echo ''" C-m
tmux send-keys "echo 'Checking node status...'" C-m
tmux send-keys "ros2 node list" C-m
tmux send-keys "echo ''" C-m
tmux send-keys "echo 'If all nodes are running, you can now:'" C-m
tmux send-keys "echo '  1. Set initial pose in RViz'" C-m
tmux send-keys "echo '  2. Send navigation goals'" C-m
tmux send-keys "echo ''" C-m
tmux send-keys "echo 'Type commands below:'" C-m

# Select the monitor pane so user sees the status
tmux select-pane -t 11

print_success "Tmux session created and components are launching!"
echo ""
print_header "Launch Status"
echo "✓ All Nav2 components are starting automatically"
echo "✓ Estimated ready time: ~20 seconds"
echo ""
print_header "Next Steps"
echo "1. Attach to the tmux session to monitor progress:"
echo ""
echo "   ${GREEN}tmux attach -t $SESSION_NAME${NC}"
echo ""
echo "2. After ~20 seconds, in RViz:"
echo "   - Set initial pose (2D Pose Estimate)"
echo "   - Send navigation goal (Nav2 Goal)"
echo ""
echo "3. Navigate between panes: ${YELLOW}Ctrl+b then arrow keys${NC}"
echo "4. Zoom into a pane: ${YELLOW}Ctrl+b then z${NC}"
echo ""
print_warning "IMPORTANT: Make sure your robot hardware is running on the Pi first!"
echo "   On Pi: ./src/ros_arduino_bridge/deployment/scripts/pi/01_arduino_only.sh"
echo ""
print_info "To kill this session later: tmux kill-session -t $SESSION_NAME"
print_info "To run diagnostics: ./navigation_monitor.sh full"
echo ""

# Auto-attach to session
read -p "Attach to tmux session now? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    tmux attach -t $SESSION_NAME
fi
