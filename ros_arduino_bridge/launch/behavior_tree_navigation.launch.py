#!/usr/bin/env python3
"""
Launch file for py_trees behavior tree navigation
This is the EXACT state machine copied from gazebo_ignition_fortress project
"""

import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch the py_trees behavior tree for autonomous navigation
    
    This launch file runs the behavior tree state machine that:
    1. Moves robot to waypoint 1 (2.1, 0.0)
    2. Executes task 1 (prints "hello")
    3. Moves robot to waypoint 2 (0, -1.2)
    4. Executes task 2 (prints "hi")
    
    The state machine uses Nav2's NavigateToPose action and AMCL localization.
    """
    
    # Get the package directory dynamically
    pkg_dir = get_package_share_directory('ros_arduino_bridge')
    # Navigate to the behavior tree script (it's in the source, not install)
    # We need to find the source directory from the install directory
    install_dir = Path(pkg_dir)
    workspace_root = install_dir.parent.parent.parent  # Go up from install/ros_arduino_bridge/share/ros_arduino_bridge
    behavior_tree_script = workspace_root / 'src' / 'ros_arduino_bridge' / 'behavior_tree' / 'robot_navigation_bt.py'
    
    # Fallback to common locations if not found
    if not behavior_tree_script.exists():
        # Try alternative paths
        alt_paths = [
            Path.home() / 'ros2_ws' / 'src' / 'ros_arduino_bridge' / 'behavior_tree' / 'robot_navigation_bt.py',
            Path('/home/allen-wachio/ros2_ws/src/ros_arduino_bridge/behavior_tree/robot_navigation_bt.py'),
            Path('/root/ros2_ws/src/robotics_dojo_project/ros_arduino_bridge/behavior_tree/robot_navigation_bt.py'),
        ]
        for alt_path in alt_paths:
            if alt_path.exists():
                behavior_tree_script = alt_path
                break
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        
        # Execute the behavior tree Python script directly
        # This is the EXACT code from gazebo_ignition_fortress/test_folder/app.py
        ExecuteProcess(
            cmd=['python3', str(behavior_tree_script)],
            output='screen',
            shell=False,
            emulate_tty=True,
        ),
    ])
