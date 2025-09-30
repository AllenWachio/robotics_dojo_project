#!/bin/bash

# LAPTOP SCRIPT - Map Saver
# Use this to save maps after mapping session

echo "Saving current map..."
echo "Make sure mapping is currently running!"
echo ""

# Create maps directory if it doesn't exist
mkdir -p ~/ros2_ws/maps

# Function to check if map name exists
check_map_exists() {
    local map_name="$1"
    if [ -f ~/ros2_ws/maps/${map_name}.yaml ] || [ -f ~/ros2_ws/maps/${map_name}.pgm ]; then
        return 0  # exists
    else
        return 1  # doesn't exist
    fi
}

# Get map name from user
while true; do
    echo "Enter a name for your map (no spaces, letters/numbers/underscores only):"
    read -r map_name
    
    # Check if name is empty
    if [ -z "$map_name" ]; then
        echo "❌ Map name cannot be empty. Please try again."
        continue
    fi
    
    # Check if name contains invalid characters
    if [[ ! "$map_name" =~ ^[a-zA-Z0-9_]+$ ]]; then
        echo "❌ Invalid characters in name. Use only letters, numbers, and underscores."
        continue
    fi
    
    # Check if map already exists
    if check_map_exists "$map_name"; then
        echo "❌ A map named '$map_name' already exists!"
        echo "Existing maps in ~/ros2_ws/maps/:"
        ls -1 ~/ros2_ws/maps/*.yaml 2>/dev/null | sed 's|.*/||' | sed 's|\.yaml||' || echo "  (no maps found)"
        echo ""
        echo "Please choose a different name."
        continue
    fi
    
    # Valid name that doesn't exist
    break
done

echo ""
echo "Saving map as: $map_name"
echo "Location: ~/ros2_ws/maps/"

# Save the map
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/$map_name

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Map saved successfully!"
    echo "Files created:"
    echo "  - ~/ros2_ws/maps/${map_name}.yaml"
    echo "  - ~/ros2_ws/maps/${map_name}.pgm"
    echo ""
    echo "To use this map for navigation:"
    echo "  ./02_navigation_mode.sh $map_name"
    echo ""
    echo "All saved maps:"
    ls -1 ~/ros2_ws/maps/*.yaml 2>/dev/null | sed 's|.*/||' | sed 's|\.yaml||' | while read map; do
        echo "  - $map"
    done
else
    echo "❌ Failed to save map. Make sure SLAM is running."
fi