"""
Launch file for sensor fusion using robot_localization EKF.
Fuses wheel odometry + IMU data to produce accurate pose estimates
that correct for wheel slippage during turns.

Usage (on Raspberry Pi):
    ros2 launch ros_arduino_bridge sensor_fusion.launch.py

This launch file:
1. Publishes static transform: base_link → imu_link (IMU position)
2. Starts the robot_localization EKF node
3. Loads configuration from ekf_config.yaml
4. Subscribes to /odom (wheels) and /imu/data (IMU)
5. Publishes fused odometry to /odometry/filtered
6. EKF publishes TF transform (odom → base_link)

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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the path to the config file
    pkg_share = get_package_share_directory('ros_arduino_bridge')
    ekf_config_path = os.path.join(pkg_share, 'config', 'ekf_config.yaml')
    
    # Declare launch arguments
    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf',
        default_value='true',
        description='Enable EKF sensor fusion'
    )
    
    # Static Transform: base_link → imu_link
    # IMPORTANT: Adjust x, y, z to match your IMU's actual position on the robot!
    # Current values: IMU is UNDER base_link (negative z)
    # 
    # Coordinate system (from robot's perspective):
    #   x = forward/backward (+ forward, - backward)
    #   y = left/right (+ left, - right)
    #   z = up/down (+ up, - down)
    #
    # Example measurements:
    #   - IMU 3cm below base_link center → z = -0.03
    #   - IMU 5cm below, 2cm forward → x = 0.02, z = -0.05
    #
    # TODO: MEASURE YOUR ACTUAL IMU POSITION AND UPDATE THESE VALUES!
    imu_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_tf',
        arguments=['0', '0', '-0.03', '0', '0', '0', 'base_link', 'imu_link'],
        #          x    y    z=-3cm  roll pitch yaw
        output='screen'
    )
    
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
        use_ekf_arg,
        imu_static_tf,
        ekf_node
    ])
