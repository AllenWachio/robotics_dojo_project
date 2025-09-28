#!/bin/bash
# LAPTOP SETUP SCRIPT
# Sets up your laptop as the base station for distributed robot computing

echo "🖥️  LAPTOP BASE STATION SETUP"
echo "=================================="
echo ""

# Get network information
LAPTOP_IP=$(hostname -I | awk '{print $1}')
echo "Laptop IP: $LAPTOP_IP"

# Get robot IP
read -p "Enter Raspberry Pi robot IP: " PI_IP
if [[ ! $PI_IP =~ ^[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}$ ]]; then
    echo "❌ Invalid IP format"
    exit 1
fi

echo "Robot IP: $PI_IP"
echo ""

# Configure ROS2 networking
echo "🔧 Configuring ROS2 networking..."
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0

# Make networking persistent
echo "# Distributed Robot Computing - Laptop Base Station" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc

# Create maps directory
mkdir -p ~/robot_maps

# Create startup script
cat > ~/start_base_station.sh << EOF
#!/bin/bash
echo "🚀 Starting Robot Base Station..."
echo ""

# ROS2 Environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Networking
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

echo "Connecting to robot at $PI_IP..."
echo "Starting SLAM processing and visualization..."
echo ""

# Launch base station  
cd ~/ros2_ws/src/ros_arduino_bridge/deployment/laptop
ros2 launch . laptop_base_station.launch.py

EOF
chmod +x ~/start_base_station.sh

# Create connection test script
cat > ~/test_robot_connection.sh << EOF
#!/bin/bash
echo "🔍 Testing Robot Connection..."
echo "Robot IP: $PI_IP"
echo ""

# Setup environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

echo "Checking network connectivity..."
if ping -c 2 $PI_IP > /dev/null 2>&1; then
    echo "✅ Network: Robot reachable"
else
    echo "❌ Network: Cannot reach robot"
    exit 1
fi

echo ""
echo "Checking for robot topics..."
timeout 10s ros2 topic list 2>/dev/null | grep -E "(odom|scan|tf|joint_states)" > /tmp/robot_topics

if [ -s /tmp/robot_topics ]; then
    echo "✅ ROS2: Robot topics detected"
    echo ""
    echo "Available robot topics:"
    cat /tmp/robot_topics
    echo ""
    echo "🎉 Connection successful! You can start the base station."
else
    echo "❌ ROS2: No robot topics found"
    echo ""
    echo "Troubleshooting:"
    echo "1. Ensure robot is running: ssh pi@$PI_IP './start_robot.sh'"
    echo "2. Check ROS_DOMAIN_ID matches on both machines"
    echo "3. Verify firewall settings"
fi

rm -f /tmp/robot_topics
EOF
chmod +x ~/test_robot_connection.sh

# Create map saving script
cat > ~/save_robot_map.sh << 'EOF'
#!/bin/bash
echo "💾 Saving Robot Map..."

# Setup environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# Get map name
read -p "Enter map name (default: robot_map): " MAP_NAME
MAP_NAME=${MAP_NAME:-robot_map}

# Save to maps directory
cd ~/robot_maps
ros2 run nav2_map_server map_saver_cli -f "$MAP_NAME"

if [ $? -eq 0 ]; then
    echo "✅ Map saved as ~/robot_maps/$MAP_NAME.yaml"
    echo "   Map image: ~/robot_maps/$MAP_NAME.pgm"
    echo ""
    echo "🗺️  Available maps:"
    ls -la ~/robot_maps/*.yaml 2>/dev/null | awk '{print "   " $9}' | sed 's|.*/||'
    echo ""
    echo "🚀 Ready for navigation! Use: ./start_navigation.sh $MAP_NAME"
else
    echo "❌ Failed to save map. Is SLAM running?"
fi
EOF
chmod +x ~/save_robot_map.sh

# Create navigation startup script  
cat > ~/start_navigation.sh << 'EOF'
#!/bin/bash
echo "🧭 Starting Robot Navigation System..."
echo ""

# Setup ROS2 environment
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Networking
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# Get map file
MAP_FILE="$1"
if [ -z "$MAP_FILE" ]; then
    echo "📋 Available maps:"
    ls ~/robot_maps/*.yaml 2>/dev/null | sed 's|.*/||; s|\.yaml||' | awk '{print "   " $1}'
    echo ""
    read -p "Enter map name (without .yaml): " MAP_FILE
fi

if [ -z "$MAP_FILE" ]; then
    MAP_FILE="robot_map"
fi

# Check if map exists
if [ ! -f ~/robot_maps/${MAP_FILE}.yaml ]; then
    echo "❌ Map ~/robot_maps/${MAP_FILE}.yaml not found!"
    echo "📋 Available maps:"
    ls ~/robot_maps/*.yaml 2>/dev/null | sed 's|.*/||; s|\.yaml||' | awk '{print "   " $1}'
    exit 1
fi

echo "🗺️  Loading map: $MAP_FILE"
echo "🤖 Connecting to robot..."
echo "🧭 Starting autonomous navigation..."
echo ""
echo "📖 Navigation Instructions:"
echo "1. Set initial pose using '2D Pose Estimate' tool in RViz"
echo "2. Set navigation goal using '2D Nav Goal' tool"
echo "3. Robot will plan path and navigate autonomously"
echo ""

# Launch navigation system
cd ~/ros2_ws/src/ros_arduino_bridge/deployment/laptop
ros2 launch . laptop_navigation.launch.py map_file:=${MAP_FILE}.yaml

EOF
chmod +x ~/start_navigation.sh

# Create map management script
cat > ~/manage_maps.sh << 'EOF'
#!/bin/bash
echo "🗺️  ROBOT MAP MANAGER"
echo "===================="
echo ""

cd ~/robot_maps

# Show available maps
echo "📋 Available Maps:"
if ls *.yaml &> /dev/null; then
    for map_file in *.yaml; do
        map_name="${map_file%.yaml}"
        if [ -f "${map_name}.pgm" ]; then
            map_size=$(identify "${map_name}.pgm" 2>/dev/null | awk '{print $3}' || echo "unknown")
            echo "   ✅ $map_name (${map_size})"
        else
            echo "   ⚠️  $map_name (missing .pgm file)"
        fi
    done
else
    echo "   (No maps found)"
fi

echo ""
echo "🔧 Commands:"
echo "1. View map: eog <map_name>.pgm"
echo "2. Delete map: rm <map_name>.yaml <map_name>.pgm"
echo "3. Copy map: cp <map_name>.* <new_name>.*"
echo "4. Start navigation: ~/start_navigation.sh <map_name>"
echo ""

# Interactive commands
while true; do
    read -p "Enter command (view/delete/copy/nav/quit): " cmd
    case $cmd in
        view|v)
            read -p "Map name to view: " map_name
            if [ -f "${map_name}.pgm" ]; then
                eog "${map_name}.pgm" &
                echo "✅ Opened ${map_name}.pgm"
            else
                echo "❌ Map ${map_name}.pgm not found"
            fi
            ;;
        delete|d)
            read -p "Map name to delete: " map_name
            if [ -f "${map_name}.yaml" ]; then
                read -p "⚠️  Delete ${map_name}? (y/N): " confirm
                if [[ $confirm =~ ^[Yy]$ ]]; then
                    rm -f "${map_name}.yaml" "${map_name}.pgm"
                    echo "✅ Deleted ${map_name}"
                fi
            else
                echo "❌ Map ${map_name} not found"
            fi
            ;;
        copy|c)
            read -p "Source map name: " src_name
            read -p "New map name: " dst_name
            if [ -f "${src_name}.yaml" ]; then
                cp "${src_name}.yaml" "${dst_name}.yaml"
                cp "${src_name}.pgm" "${dst_name}.pgm" 2>/dev/null
                echo "✅ Copied ${src_name} to ${dst_name}"
            else
                echo "❌ Source map ${src_name} not found"
            fi
            ;;
        nav|n)
            read -p "Map name for navigation: " map_name
            if [ -f "${map_name}.yaml" ]; then
                echo "🚀 Starting navigation with ${map_name}..."
                ~/start_navigation.sh "$map_name"
                break
            else
                echo "❌ Map ${map_name}.yaml not found"
            fi
            ;;
        quit|q)
            break
            ;;
        *)
            echo "❌ Unknown command: $cmd"
            ;;
    esac
    echo ""
done
EOF
chmod +x ~/manage_maps.sh

echo ""
echo "✅ Laptop base station setup complete!"
echo ""
echo "�️  MAPPING & NAVIGATION WORKFLOW:"
echo "=================================="
echo ""
echo "📋 Phase 1 - Mapping:"
echo "1. Test connection:     ./test_robot_connection.sh"
echo "2. Start SLAM mapping:  ./start_base_station.sh"  
echo "3. Drive robot around room (teleop)"
echo "4. Save map:            ./save_robot_map.sh"
echo ""
echo "� Phase 2 - Navigation:"
echo "1. Start navigation:    ./start_navigation.sh <map_name>"
echo "2. Set initial pose in RViz (2D Pose Estimate)"
echo "3. Set navigation goals (2D Nav Goal)"
echo "4. Watch autonomous navigation!"
echo ""
echo "📂 Scripts Created:"
echo "   ~/start_base_station.sh    - SLAM mapping mode"
echo "   ~/start_navigation.sh      - Autonomous navigation mode"
echo "   ~/test_robot_connection.sh - Connection testing"
echo "   ~/save_robot_map.sh        - Map saving utility"
echo "   ~/manage_maps.sh           - Map management tool"
echo "   ~/robot_maps/              - Map storage directory"
echo ""
echo "🌐 Network Configuration:"
echo "   Laptop IP: $LAPTOP_IP"
echo "   Robot IP:  $PI_IP"
echo "   ROS Domain: 0"
echo ""
echo "🎯 Quick Start:"
echo "   Mapping:    ./start_base_station.sh"
echo "   Navigation: ./start_navigation.sh"