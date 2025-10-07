#!/bin/bash

# LAPTOP SCRIPT - SLAM Toolbox Localization Mode
# This script runs on the laptop for localization using SLAM Toolbox with serialized maps
# This is an ALTERNATIVE to AMCL - use this with serialized SLAM Toolbox maps (.data + .posegraph)
# The Pi should be running hardware nodes (see pi scripts)
# Usage: ./02b_slam_localization_mode.sh [map_name]

echo "========================================="
echo "  SLAM Toolbox Localization Mode"
echo "========================================="
echo "This mode uses SLAM Toolbox for localization (not AMCL)"
echo "Make sure you have saved a map with BOTH formats!"
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
        echo "Options:"
        echo "  1. Run mapping mode and save with the updated 03_save_map.sh script"
        echo "  2. Use AMCL instead: ./02_navigation_mode.sh (works with old format maps)"
        exit 1
    fi
    
    echo "$available_maps" | while read map; do
        # Check if both serialized files exist
        if [ -f $HOME/ros2_ws/maps/${map}.data ] && [ -f $HOME/ros2_ws/maps/${map}.posegraph ]; then
            echo "  ✅ $map (serialized format available)"
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
            echo "❌ Serialized map '$map_name' not found. Please choose from available maps above."
            continue
        fi
    done
fi

# Verify the selected map exists (both serialized files)
if [ ! -f $HOME/ros2_ws/maps/${map_name}.data ] || [ ! -f $HOME/ros2_ws/maps/${map_name}.posegraph ]; then
    echo "❌ Serialized map files not found for '$map_name'!"
    echo "Required files:"
    echo "  - $HOME/ros2_ws/maps/${map_name}.data"
    echo "  - $HOME/ros2_ws/maps/${map_name}.posegraph"
    echo ""
    echo "Available serialized maps:"
    ls -1 $HOME/ros2_ws/maps/*.data 2>/dev/null | sed 's|.*/||' | sed 's|\.data||' || echo "  (no serialized maps found)"
    echo ""
    echo "To create serialized maps, run mapping mode and use the updated save script."
    exit 1
fi

echo "✅ Using serialized map: $map_name"
echo "Map files:"
echo "  - $HOME/ros2_ws/maps/${map_name}.data"
echo "  - $HOME/ros2_ws/maps/${map_name}.posegraph"
echo ""

# Use $HOME for ROS2 (not ~ which doesn't expand in all contexts)
map_path_expanded="$HOME/ros2_ws/maps"

# Source the workspace
source /opt/ros/humble/setup.bash
source $HOME/ros2_ws/install/setup.bash

echo "========================================="
echo "  Starting SLAM Toolbox Localization..."
echo "========================================="
echo ""
echo "📍 IMPORTANT: After RViz opens:"
echo "   1. Use '2D Pose Estimate' tool to set robot's initial position"
echo "   2. Click and drag on the map where the robot actually is"
echo "   3. Point the arrow in the direction the robot is facing"
echo "   4. SLAM Toolbox will localize the robot on the saved map"
echo ""
echo "Once localized, you can:"
echo "   - Use '2D Goal Pose' to navigate autonomously"
echo "   - Or launch full Nav2 stack separately for advanced navigation"
echo ""

# Launch SLAM Toolbox localization on laptop with specified map
ros2 launch ros_arduino_bridge laptop_slam_localization.launch.py \
    map_file:=${map_name} \
    map_path:=${map_path_expanded}/
