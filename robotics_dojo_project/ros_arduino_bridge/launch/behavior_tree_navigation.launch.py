#!/usr/bin/env python3
"""
Launch file for py_trees behavior tree navigation
This is the EXACT state machine copied from gazebo_ignition_fortress project
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
            cmd=['python3', 
                 '/root/ros2_ws/src/robotics_dojo_project/ros_arduino_bridge/behavior_tree/robot_navigation_bt.py'],
            output='screen',
            shell=False,
            emulate_tty=True,
        ),
    ])
