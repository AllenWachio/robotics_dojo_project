"""
Complete launch file for Arduino bridge with EKF sensor fusion.
This launches both the Arduino bridge and EKF node together with proper configuration.

Usage (on Raspberry Pi):
    ros2 launch ros_arduino_bridge arduino_bridge_with_ekf.launch.py

This launch file:
1. Starts ros_arduino_bridge node with TF publishing DISABLED
2. Starts robot_localization EKF node with TF publishing ENABLED
3. Bridge publishes: /odom (raw) and /imu/data
4. EKF publishes: /odometry/filtered (fused) and TF transform

Result: Clean TF tree with accurate, fused odometry that corrects wheel slippage.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package paths
    pkg_share = get_package_share_directory('ros_arduino_bridge')
    
    # Config file paths
    robot_params_path = os.path.join(pkg_share, 'config', 'robot_params.yaml')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf_config.yaml')
    
    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
        description='Arduino serial port'
    )
    
    # Arduino Bridge Node (with TF disabled - EKF will publish TF)
    arduino_bridge_node = Node(
        package='ros_arduino_bridge',
        executable='ros_arduino_bridge',
        name='ros_arduino_bridge',
        output='screen',
        parameters=[
            robot_params_path,
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'publish_tf': False  # ← CRITICAL: Disable TF, let EKF handle it
            }
        ]
    )
    
    # EKF Node for Sensor Fusion (with TF enabled)
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )
    
    return LaunchDescription([
        serial_port_arg,
        arduino_bridge_node,
        ekf_node
    ])
