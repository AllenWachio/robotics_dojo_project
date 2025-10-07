#!/usr/bin/env python3
"""
Complete SLAM Launch with EKF Sensor Fusion
============================================

This launch file properly integrates:
1. Arduino Bridge (provides /odom and /imu/data, TF disabled)
2. EKF Sensor Fusion (fuses encoders + IMU → /odometry/filtered, publishes TF)
3. SLAM Toolbox (uses filtered odometry for mapping)
4. Robot State Publisher (publishes robot model)

CRITICAL: This fixes the "map breaks during turns" issue by:
- Using IMU-corrected odometry instead of raw encoder odometry
- Avoiding TF publishing conflicts
- Proper sensor fusion configuration

Usage:
    ros2 launch ros_arduino_bridge slam_with_sensor_fusion.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get package paths
    pkg_share = FindPackageShare('ros_arduino_bridge')
    
    # Configuration files
    slam_params_file = PathJoinSubstitution([
        pkg_share, 'config', 'mapper_params_online_async.yaml'
    ])
    ekf_params_file = PathJoinSubstitution([
        pkg_share, 'config', 'ekf_config.yaml'
    ])
    urdf_path = PathJoinSubstitution([
        pkg_share, 'urdf', 'new_robot_urdf.xacro'
    ])

    # =====================================================================
    # Robot State Publisher - Publishes robot URDF model
    # =====================================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path]),
            'use_sim_time': False
        }]
    )

    # =====================================================================
    # Arduino Bridge - Provides /odom (encoders) and /imu/data
    # CRITICAL: publish_tf=False to avoid conflict with EKF
    # =====================================================================
    arduino_bridge = Node(
        package='ros_arduino_bridge',
        executable='ros_arduino_bridge',
        name='ros_arduino_bridge',
        output='screen',
        parameters=[{
            'serial_port': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            'baud_rate': 57600,
            'base_width': 0.249000,      # Effective track width (calibrated)
            'wheel_radius': 0.042500,    # 85mm diameter wheels
            'encoder_ticks_per_rev': 447,  # Calibrated value
            'publish_tf': False,         # ⚠️ CRITICAL: Let EKF handle TF publishing
        }]
    )

    # =====================================================================
    # EKF Sensor Fusion Node - Fuses /odom + /imu/data
    # Outputs: /odometry/filtered (corrects wheel slippage during turns)
    # Publishes: odom→base_link transform
    # =====================================================================
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file],
        remappings=[
            # No remapping needed - uses /odom and /imu/data directly
        ]
    )

    # =====================================================================
    # SLAM Toolbox Node - Creates map using filtered odometry
    # CRITICAL: Remapped to use /odometry/filtered instead of /odom
    # This ensures SLAM uses IMU-corrected odometry, fixing turn issues
    # =====================================================================
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': False}],
        remappings=[
            ('/odom', '/odometry/filtered')  # ⚠️ CRITICAL: Use filtered odometry!
        ]
    )

    # Return all nodes
    return LaunchDescription([
        robot_state_publisher,
        arduino_bridge,
        ekf_node,
        slam_toolbox_node
    ])
