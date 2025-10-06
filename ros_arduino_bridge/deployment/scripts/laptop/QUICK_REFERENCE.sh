#!/bin/bash
# Quick Reference Card - ROS2 Navigation
# =======================================

cat << 'EOF'

╔═══════════════════════════════════════════════════════════════════════╗
║                    ROS2 ROBOT NAVIGATION GUIDE                        ║
║                      Quick Reference Card                             ║
╚═══════════════════════════════════════════════════════════════════════╝

📍 LOCATION: ~/ros2_ws/src/ros_arduino_bridge/deployment/scripts/laptop

┌───────────────────────────────────────────────────────────────────────┐
│ PHASE 1: CREATE MAP                                                   │
└───────────────────────────────────────────────────────────────────────┘

  On Pi (2 terminals):
    Terminal 1: ./01_arduino_only.sh
    Terminal 2: ./02_lidar_only.sh

  On Laptop:
    ./01_mapping_mode.sh         # Drive robot around
    ./03_save_map.sh             # Save when done (creates 4 files!)

┌───────────────────────────────────────────────────────────────────────┐
│ PHASE 2: NAVIGATE WITH MAP                                            │
└───────────────────────────────────────────────────────────────────────┘

  Choose ONE navigation method:

  ┌─────────────────────────────────────────────────────────────────────┐
  │ Option A: AMCL Navigation (Traditional)                             │
  ├─────────────────────────────────────────────────────────────────────┤
  │   ./02_navigation_mode.sh [map_name]                                │
  │   • Uses AMCL localization                                          │
  │   • Industry standard, well-tested                                  │
  │   • Works with .pgm/.yaml maps                                      │
  └─────────────────────────────────────────────────────────────────────┘

  ┌─────────────────────────────────────────────────────────────────────┐
  │ Option B: SLAM Localization Only (Testing)                          │
  ├─────────────────────────────────────────────────────────────────────┤
  │   ./02b_slam_localization_mode.sh [map_name]                        │
  │   • Localization only, no navigation                                │
  │   • Good for testing map quality                                    │
  │   • Uses .data/.posegraph maps                                      │
  └─────────────────────────────────────────────────────────────────────┘

  ┌─────────────────────────────────────────────────────────────────────┐
  │ Option C: SLAM Navigation ⭐ RECOMMENDED!                           │
  ├─────────────────────────────────────────────────────────────────────┤
  │   ./02c_slam_navigation_mode.sh [map_name]                          │
  │   • BEST ACCURACY - Most precise localization                       │
  │   • SLAM Toolbox + Nav2                                             │
  │   • Can refine map while navigating                                 │
  │   • Uses .data/.posegraph maps                                      │
  └─────────────────────────────────────────────────────────────────────┘

┌───────────────────────────────────────────────────────────────────────┐
│ IN RVIZ (After navigation starts)                                     │
└───────────────────────────────────────────────────────────────────────┘

  1. Set Initial Pose:
     • Click "2D Pose Estimate" button
     • Click on map where robot is
     • Drag to set robot orientation
     • Wait for localization to stabilize (~5 seconds)

  2. Set Navigation Goal:
     • Click "2D Goal Pose" or "Nav2 Goal" button
     • Click destination on map
     • Drag to set final orientation
     • Robot navigates automatically!

┌───────────────────────────────────────────────────────────────────────┐
│ MAP FORMATS EXPLAINED                                                  │
└───────────────────────────────────────────────────────────────────────┘

  When you save a map, you get 4 files:

  📄 Old Format (AMCL):          📄 Serialized (SLAM Toolbox):
     map_name.pgm                   map_name.data
     map_name.yaml                  map_name.posegraph

  Location: ~/ros2_ws/maps/

┌───────────────────────────────────────────────────────────────────────┐
│ TROUBLESHOOTING                                                        │
└───────────────────────────────────────────────────────────────────────┘

  Map doesn't show in RViz:
    • Check map files exist: ls ~/ros2_ws/maps/
    • Verify RViz map topic: Durability = "Transient Local"
    • Check map_server running: ros2 node list

  Robot doesn't move to goal:
    • Verify localization first (small pose covariance)
    • Check cmd_vel: ros2 topic echo /cmd_vel
    • Check lifecycle: ros2 lifecycle list
    • Add costmap visualization in RViz

  Serialized map save fails:
    • SLAM Toolbox must be running during save
    • Check service: ros2 service list | grep serialize
    • Verify permissions: ls -l ~/ros2_ws/maps/

┌───────────────────────────────────────────────────────────────────────┐
│ USEFUL COMMANDS                                                        │
└───────────────────────────────────────────────────────────────────────┘

  List saved maps:
    ls -lh ~/ros2_ws/maps/

  Check running nodes:
    ros2 node list

  View TF tree:
    ros2 run tf2_tools view_frames.py

  Check topics:
    ros2 topic list
    ros2 topic echo /odom
    ros2 topic echo /scan

  Monitor navigation:
    ros2 topic echo /cmd_vel
    ros2 topic echo /plan

  Check lifecycle status:
    ros2 lifecycle list
    ros2 lifecycle get <node_name>

┌───────────────────────────────────────────────────────────────────────┐
│ MORE HELP                                                              │
└───────────────────────────────────────────────────────────────────────┘

  📚 Full Documentation:
     ~/ros2_ws/src/ros_arduino_bridge/deployment/scripts/COMPLETE_WORKFLOW.md
     ~/ros2_ws/src/ros_arduino_bridge/deployment/scripts/NAVIGATION_SETUP.md

  🔧 Other Scripts:
     ./04_diagnostics.sh          # System health check

╔═══════════════════════════════════════════════════════════════════════╗
║  TIP: Start with Option C (SLAM Navigation) for best results! 🚀     ║
╚═══════════════════════════════════════════════════════════════════════╝

EOF
