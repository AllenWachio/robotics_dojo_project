#!/usr/bin/env python3
"""
Launch file for incremental navigation test mission.
Runs the test behavior tree for room mapping and navigation.

Author: Robotics Dojo 2025
Date: October 6, 2025
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path
import os


def generate_launch_description():
    """Generate launch description for incremental navigation test."""
    
    # Get package directory
    package_dir = None
    
    # Try multiple methods to find package directory
    try:
        from ament_index_python.packages import get_package_share_directory
        package_dir = get_package_share_directory('ros_arduino_bridge')
    except:
        # Fallback: use path relative to this file
        current_file = Path(__file__).resolve()
        package_dir = str(current_file.parent.parent)
    
    # Define behavior tree script path
    bt_script = os.path.join(package_dir, 'behavior_tree', 'test_incremental_navigation.py')
    
    # Ensure Python can find our modules
    behavior_tree_dir = os.path.join(package_dir, 'behavior_tree')
    
    # Create node for behavior tree
    behavior_tree_node = Node(
        package='ros_arduino_bridge',
        executable='test_incremental_navigation.py',
        name='test_incremental_navigation',
        output='screen',
        parameters=[],
        additional_env={'PYTHONPATH': f"{behavior_tree_dir}:{os.environ.get('PYTHONPATH', '')}"}
    )
    
    return LaunchDescription([
        behavior_tree_node
    ])
