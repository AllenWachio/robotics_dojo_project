#!/bin/bash

# LAPTOP SCRIPT - Map Saver
# Use this to save maps after mapping session

echo "Saving current map..."
echo "Make sure mapping is currently running!"
echo ""

# Create maps directory if it doesn't exist
mkdir -p ~/ros2_ws/maps

# Save the map
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/my_map

if [ $? -eq 0 ]; then
    echo "Map saved successfully to ~/ros2_ws/maps/my_map"
    echo "Files created:"
    echo "  - ~/ros2_ws/maps/my_map.yaml"
    echo "  - ~/ros2_ws/maps/my_map.pgm"
else
    echo "Failed to save map. Make sure SLAM is running."
fi