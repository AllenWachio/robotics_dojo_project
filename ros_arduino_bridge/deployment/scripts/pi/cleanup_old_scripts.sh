#!/bin/bash

# Script to clean up redundant files after switching to separated approach

echo "Cleaning up redundant scripts..."
echo "The new separated approach uses:"
echo "  - 01_arduino_only.sh (instead of 01_hardware_interface.sh)"
echo "  - 02_lidar_only.sh (includes troubleshooting)"
echo ""

read -p "Remove old scripts? (y/N): " -n 1 -r
echo

if [[ $REPLY =~ ^[Yy]$ ]]; then
    cd ~/ros2_ws/src/robotics_dojo_project/ros_arduino_bridge/deployment/scripts/pi
    
    echo "Backing up old scripts to old_scripts/ directory..."
    mkdir -p old_scripts
    
    if [ -f "01_hardware_interface.sh" ]; then
        mv 01_hardware_interface.sh old_scripts/
        echo "  ✓ Moved 01_hardware_interface.sh to backup"
    fi
    
    if [ -f "04_lidar_troubleshoot.sh" ]; then
        mv 04_lidar_troubleshoot.sh old_scripts/
        echo "  ✓ Moved 04_lidar_troubleshoot.sh to backup"
    fi
    
    echo ""
    echo "Cleanup complete! Old scripts backed up in old_scripts/"
    echo ""
    echo "Current active scripts:"
    ls -1 *.sh | grep -E "(01_arduino_only|02_lidar_only|fix_usb_permissions|02_setup_test|03_network_config)"
else
    echo "Cleanup cancelled. All scripts retained."
fi

echo ""
echo "See COMPLETE_WORKFLOW.md for the new workflow steps."