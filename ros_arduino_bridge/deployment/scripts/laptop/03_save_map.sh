#!/bin/bash

# LAPTOP SCRIPT - Map Saver
# Use this to save maps after mapping session

echo "Saving current map..."
echo "Make sure mapping is currently running!"
echo ""

# Create maps directory if it doesn't exist
mkdir -p $HOME/ros2_ws/maps

# Function to check if map name exists
check_map_exists() {
    local map_name="$1"
    if [ -f $HOME/ros2_ws/maps/${map_name}.yaml ] || [ -f $HOME/ros2_ws/maps/${map_name}.pgm ]; then
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
        echo "Existing maps in $HOME/ros2_ws/maps/:"
        ls -1 $HOME/ros2_ws/maps/*.yaml 2>/dev/null | sed 's|.*/||' | sed 's|\.yaml||' || echo "  (no maps found)"
        echo ""
        echo "Please choose a different name."
        continue
    fi
    
    # Valid name that doesn't exist
    break
done

echo ""
echo "Saving map as: $map_name"
echo "Location: $HOME/ros2_ws/maps/"
echo ""
echo "==================== SAVING MAPS ===================="
echo "Saving in BOTH formats:"
echo "  1. Old format (.pgm + .yaml) - for AMCL/Nav2"
echo "  2. Serialized format (.data + .posegraph) - for SLAM Toolbox localization"
echo "====================================================="
echo ""

# Save the old format map (.pgm + .yaml) - for AMCL and Nav2
echo "📁 Step 1/2: Saving old format map..."
ros2 run nav2_map_server map_saver_cli -f $HOME/ros2_ws/maps/$map_name

if [ $? -eq 0 ]; then
    echo "✅ Old format map saved!"
    echo "  - $HOME/ros2_ws/maps/${map_name}.yaml"
    echo "  - $HOME/ros2_ws/maps/${map_name}.pgm"
else
    echo "❌ Failed to save old format map. Make sure SLAM is running."
    exit 1
fi

echo ""
echo "📁 Step 2/2: Saving serialized map (SLAM Toolbox format)..."

# Save serialized map (.data + .posegraph) - for SLAM Toolbox localization
# This uses the slam_toolbox serialize_map service
# IMPORTANT: Must use $HOME or absolute path - ROS services don't expand ~
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "{filename: '$HOME/ros2_ws/maps/${map_name}'}"

# Check if serialized files were created
sleep 2  # Give time for files to be written

if [ -f $HOME/ros2_ws/maps/${map_name}.data ] && [ -f $HOME/ros2_ws/maps/${map_name}.posegraph ]; then
    echo "✅ Serialized map saved!"
    echo "  - $HOME/ros2_ws/maps/${map_name}.data"
    echo "  - $HOME/ros2_ws/maps/${map_name}.posegraph"
    echo ""
    echo "==================== SUCCESS ===================="
    echo "✅ All 4 map files saved successfully!"
    echo ""
    echo "📋 Files created:"
    echo "  Old format (for AMCL/Nav2):"
    echo "    - $HOME/ros2_ws/maps/${map_name}.yaml"
    echo "    - $HOME/ros2_ws/maps/${map_name}.pgm"
    echo "  Serialized format (for SLAM Toolbox localization):"
    echo "    - $HOME/ros2_ws/maps/${map_name}.data"
    echo "    - $HOME/ros2_ws/maps/${map_name}.posegraph"
    echo ""
    echo "================================================="
    echo ""
    echo "📍 To use this map for navigation:"
    echo "  Option 1 (AMCL): ./02_navigation_mode.sh $map_name"
    echo "  Option 2 (SLAM Localization): ./02b_slam_localization_mode.sh $map_name"
    echo ""
    echo "📂 All saved maps:"
    ls -1 $HOME/ros2_ws/maps/*.yaml 2>/dev/null | sed 's|.*/||' | sed 's|\.yaml||' | while read map; do
        # Check if both formats exist
        if [ -f $HOME/ros2_ws/maps/${map}.data ]; then
            echo "  - $map (✅ both formats)"
        else
            echo "  - $map (⚠️  old format only)"
        fi
    done
else
    echo "⚠️  Warning: Serialized map files not found!"
    echo "Old format map was saved, but serialization may have failed."
    echo "This could happen if:"
    echo "  1. SLAM Toolbox is not running"
    echo "  2. The serialize_map service is not available"
    echo ""
    echo "You can still use the old format map with AMCL:"
    echo "  ./02_navigation_mode.sh $map_name"
    echo ""
    echo "To use serialized maps in the future, make sure SLAM Toolbox"
    echo "is running when you save the map."
fi