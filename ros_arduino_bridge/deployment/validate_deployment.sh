#!/bin/bash
# DEPLOYMENT VALIDATION SCRIPT
# Checks if all deployment files are correct and functional

echo "🔍 DEPLOYMENT VALIDATION"
echo "========================="
echo ""

cd "$(dirname "$0")"

# Check file structure
echo "📁 Checking deployment structure..."
MISSING=0

# Essential files
REQUIRED_FILES=(
    "laptop/laptop_base_station.launch.py"
    "laptop/laptop_navigation.launch.py"
    "laptop/slam_config_laptop.yaml"
    "laptop/nav2_params.yaml" 
    "laptop/laptop_rviz_config.rviz"
    "laptop/navigation_rviz_config.rviz"
    "laptop/setup_laptop.sh"
    "pi/pi_robot_hardware.launch.py"
    "pi/setup_pi.sh"
    "shared/robot_description.xacro"
    "shared/materials.xacro"
    "shared/new_robot_urdf.trans"
    "shared/new_robot_urdf.gazebo"
    "shared/README.md"
)

for file in "${REQUIRED_FILES[@]}"; do
    if [ -f "$file" ]; then
        echo "✅ $file"
    else
        echo "❌ $file - MISSING"
        MISSING=$((MISSING + 1))
    fi
done

echo ""

# Check executable permissions
echo "🔧 Checking script permissions..."
for script in laptop/setup_laptop.sh pi/setup_pi.sh; do
    if [ -x "$script" ]; then
        echo "✅ $script - executable"
    else
        echo "❌ $script - not executable"
        chmod +x "$script" 2>/dev/null && echo "   → Fixed permissions"
    fi
done

echo ""

# Validate Python syntax
echo "🐍 Checking Python launch files..."
python3 -m py_compile laptop/laptop_base_station.launch.py 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✅ laptop_base_station.launch.py - syntax OK"
else
    echo "❌ laptop_base_station.launch.py - syntax error"
    MISSING=$((MISSING + 1))
fi

python3 -m py_compile pi/pi_robot_hardware.launch.py 2>/dev/null  
if [ $? -eq 0 ]; then
    echo "✅ pi_robot_hardware.launch.py - syntax OK"
else
    echo "❌ pi_robot_hardware.launch.py - syntax error"
    MISSING=$((MISSING + 1))
fi

echo ""

# Check YAML files
echo "📄 Checking YAML configuration..."
python3 -c "import yaml; yaml.safe_load(open('laptop/slam_config_laptop.yaml'))" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✅ slam_config_laptop.yaml - valid YAML"
else
    echo "❌ slam_config_laptop.yaml - invalid YAML"
    MISSING=$((MISSING + 1))
fi

python3 -c "import yaml; yaml.safe_load(open('laptop/nav2_params.yaml'))" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✅ nav2_params.yaml - valid YAML"
else
    echo "❌ nav2_params.yaml - invalid YAML"
    MISSING=$((MISSING + 1))
fi

echo ""

# Check URDF
echo "🤖 Checking robot description..."
if command -v xacro >/dev/null 2>&1; then
    xacro shared/robot_description.xacro >/dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "✅ robot_description.xacro - valid URDF"
    else
        echo "❌ robot_description.xacro - URDF errors"
        MISSING=$((MISSING + 1))
    fi
else
    echo "⚠️  xacro not available - cannot validate URDF"
fi

echo ""

# Check ROS2 dependencies
echo "📦 Checking ROS2 dependencies..."
DEPS_OK=0
TOTAL_DEPS=0

check_package() {
    TOTAL_DEPS=$((TOTAL_DEPS + 1))
    if ros2 pkg list 2>/dev/null | grep -q "^$1$"; then
        echo "✅ $1"
        DEPS_OK=$((DEPS_OK + 1))
    else
        echo "❌ $1 - not installed"
    fi
}

if command -v ros2 >/dev/null 2>&1; then
    source /opt/ros/humble/setup.bash 2>/dev/null
    # SLAM dependencies
    check_package "slam_toolbox"
    check_package "robot_state_publisher"
    check_package "rviz2"
    check_package "teleop_twist_keyboard"
    check_package "sllidar_ros2"
    # Navigation dependencies
    check_package "nav2_map_server"
    check_package "nav2_amcl"
    check_package "nav2_controller"
    check_package "nav2_planner"
    check_package "nav2_behaviors"
    check_package "nav2_bt_navigator"
    check_package "nav2_lifecycle_manager"
    check_package "nav2_costmap_2d"
    check_package "nav2_velocity_smoother"
    check_package "nav2_waypoint_follower"
else
    echo "❌ ROS2 not available in PATH"
    TOTAL_DEPS=14
fi

echo ""

# Summary
echo "📊 VALIDATION SUMMARY"
echo "===================="
if [ $MISSING -eq 0 ]; then
    echo "✅ All deployment files present and valid!"
else
    echo "❌ Found $MISSING issues - fix before deployment"
fi

if [ $TOTAL_DEPS -gt 0 ]; then
    echo "📦 Dependencies: $DEPS_OK/$TOTAL_DEPS available"
    if [ $DEPS_OK -lt $TOTAL_DEPS ]; then
        echo "   Install missing packages with: sudo apt install ros-humble-<package-name>"
    fi
fi

echo ""
echo "🚀 Ready for deployment: $([ $MISSING -eq 0 ] && echo "YES" || echo "NO")"

exit $MISSING