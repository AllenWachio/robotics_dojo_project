#!/bin/bash

# LAPTOP SCRIPT - Navigation Mode
# This script runs on the laptop for autonomous navigation using saved maps
# The Pi should be running hardware nodes (see pi scripts)
# Usage: ./02_navigation_mode.sh [map_name]

echo "Starting Laptop Navigation Mode..."
echo "Make sure you have saved a map from mapping mode first!"
echo "Make sure the Pi is running the hardware script!"
echo ""

# Check if map name was provided as argument
map_name="$1"

# If no map name provided, show available maps and ask user
if [ -z "$map_name" ]; then
    echo "Available maps in ~/ros2_ws/maps/:"
    available_maps=$(ls -1 ~/ros2_ws/maps/*.yaml 2>/dev/null | sed 's|.*/||' | sed 's|\.yaml||')
    
    if [ -z "$available_maps" ]; then
        echo "❌ No maps found in ~/ros2_ws/maps/"
        echo "Please run mapping mode first and save a map."
        exit 1
    fi
    
    echo "$available_maps" | while read map; do
        echo "  - $map"
    done
    echo ""
    
    while true; do
        echo "Enter the name of the map to use:"
        read -r map_name
        
        if [ -z "$map_name" ]; then
            echo "❌ Map name cannot be empty."
            continue
        fi
        
        if [ -f ~/ros2_ws/maps/${map_name}.yaml ]; then
            break
        else
            echo "❌ Map '$map_name' not found. Please choose from available maps above."
            continue
        fi
    done
fi

# Verify the selected map exists
if [ ! -f ~/ros2_ws/maps/${map_name}.yaml ]; then
    echo "❌ Map file ~/ros2_ws/maps/${map_name}.yaml not found!"
    echo "Available maps:"
    ls -1 ~/ros2_ws/maps/*.yaml 2>/dev/null | sed 's|.*/||' | sed 's|\.yaml||' || echo "  (no maps found)"
    exit 1
fi

echo "✅ Using map: $map_name"
echo "Map file: ~/ros2_ws/maps/${map_name}.yaml"
echo ""

# Source the workspace
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

echo "Starting navigation..."

# Launch navigation on laptop with specified map
ros2 launch ros_arduino_bridge laptop_navigation.launch.py map_file:=${map_name}.yaml map_path:=~/ros2_ws/maps/