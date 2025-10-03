#!/bin/bash

# LAPTOP SCRIPT - Full Navigation with SLAM Localization
# This script runs on the laptop for autonomous navigation using SLAM Toolbox localization
# This launches BOTH localization AND Nav2 navigation stack together
# The Pi should be running hardware nodes (see pi scripts)
# Usage: ./02c_slam_navigation_mode.sh [map_name]

echo "============================================="
echo "  Full Navigation (SLAM Toolbox Localization)"
echo "============================================="
echo "This mode provides:"
echo "  - SLAM Toolbox localization (map->odom)"
echo "  - Nav2 navigation stack"
echo "  - Autonomous path planning"
echo "  - Goal-based navigation"
echo ""
echo "Make sure the Pi is running the hardware script!"
echo ""

# Check if map name was provided as argument
map_name="$1"

# If no map name provided, show available maps and ask user
if [ -z "$map_name" ]; then
    echo "Available maps in $HOME/ros2_ws/maps/:"
    
    # Find maps that have serialized format
    available_maps=$(ls -1 $HOME/ros2_ws/maps/*.data 2>/dev/null | sed 's|.*/||' | sed 's|\.data||')
    
    if [ -z "$available_maps" ]; then
        echo "❌ No serialized maps found in $HOME/ros2_ws/maps/"
        echo ""
        echo "You need serialized maps (.data + .posegraph) for SLAM Toolbox localization."
        echo "Run mapping mode and save with: ./03_save_map.sh"
        exit 1
    fi
    
    echo "$available_maps" | while read map; do
        if [ -f $HOME/ros2_ws/maps/${map}.data ] && [ -f $HOME/ros2_ws/maps/${map}.posegraph ]; then
            echo "  ✅ $map"
        fi
    done
    echo ""
    
    while true; do
        echo "Enter the name of the map to use:"
        read -r map_name
        
        if [ -z "$map_name" ]; then
            echo "❌ Map name cannot be empty."
            continue
        fi
        
        if [ -f $HOME/ros2_ws/maps/${map_name}.data ] && [ -f $HOME/ros2_ws/maps/${map_name}.posegraph ]; then
            break
        else
            echo "❌ Map '$map_name' not found. Please choose from available maps above."
            continue
        fi
    done
fi

# Verify the selected map exists
if [ ! -f $HOME/ros2_ws/maps/${map_name}.data ] || [ ! -f $HOME/ros2_ws/maps/${map_name}.posegraph ]; then
    echo "❌ Serialized map files not found for '$map_name'!"
    echo "Required files:"
    echo "  - $HOME/ros2_ws/maps/${map_name}.data"
    echo "  - $HOME/ros2_ws/maps/${map_name}.posegraph"
    exit 1
fi

echo "✅ Using map: $map_name"
echo ""

# Use $HOME for ROS2 (not ~ which doesn't expand in all contexts)
map_path_expanded="$HOME/ros2_ws/maps"

# Source the workspace
source /opt/ros/humble/setup.bash
source $HOME/ros2_ws/install/setup.bash

echo "============================================="
echo "  Starting Navigation Stack..."
echo "============================================="
echo ""
echo "📍 IMPORTANT: After RViz opens:"
echo "   1. Use '2D Pose Estimate' to set robot's initial position"
echo "   2. Wait for localization to stabilize"
echo "   3. Use '2D Goal Pose' or Nav2 Goal tool to navigate"
echo ""

# Launch full navigation with SLAM localization
ros2 launch ros_arduino_bridge laptop_navigation_slam.launch.py \
    map_file:=${map_name} \
    map_path:=${map_path_expanded}/ \
    run_localization:=true
