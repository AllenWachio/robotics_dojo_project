"""
Launch file for sensor fusion using robot_localization EKF.
Fuses wheel odometry + IMU data to produce accurate pose estimates
that correct for wheel slippage during turns.

Usage (on Raspberry Pi):
    ros2 launch ros_arduino_bridge sensor_fusion.launch.py

This launch file:
1. Starts the robot_localization EKF node
2. Loads configuration from ekf_config.yaml
3. Subscribes to /odom (wheels) and /imu/data (IMU)
4. Publishes fused odometry to /odometry/filtered
5. EKF publishes TF transform (odom → base_link)

Prerequisites:
- ros_arduino_bridge node must be running with publish_tf:=false
  (to avoid TF conflicts - EKF will publish the transform)
- robot_localization package must be installed:
    sudo apt install ros-humble-robot-localization

Example full launch:
    # Terminal 1: Arduino bridge (with TF disabled)
    ros2 launch ros_arduino_bridge arduino_bridge.py publish_tf:=false
    
    # Terminal 2: EKF sensor fusion
    ros2 launch ros_arduino_bridge sensor_fusion.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the path to the config file
    pkg_share = get_package_share_directory('ros_arduino_bridge')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf_config.yaml')
    
    # EKF node for sensor fusion
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        remappings=[
            # Input topics (from arduino bridge)
            ('/odometry/filtered', '/odometry/filtered'),  # Output topic
        ]
    )
    
    return LaunchDescription([
        ekf_node
    ])
