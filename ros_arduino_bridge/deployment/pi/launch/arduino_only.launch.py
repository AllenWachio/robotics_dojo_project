from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration, NotSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
import os

def generate_launch_description():
    """
    RASPBERRY PI ARDUINO-ONLY LAUNCH (with optional EKF)
    
    This launch file runs:
    - Arduino communication (odometry, motor control, IMU)
    - Robot state publishing (TF tree)
    - [Optional] EKF sensor fusion (fuses encoders + IMU)
    
    Usage:
        # Without EKF (default):
        ros2 launch ros_arduino_bridge arduino_only.launch.py
        
        # With EKF sensor fusion:
        ros2 launch ros_arduino_bridge arduino_only.launch.py use_ekf:=true
    
    LiDAR is launched separately to avoid conflicts
    """
    
    # Get package path for config files
    pkg_share = FindPackageShare('ros_arduino_bridge')
    
    # Launch arguments
    arduino_port = LaunchConfiguration('arduino_port')
    use_ekf = LaunchConfiguration('use_ekf')
    
    # Robot description path
    urdf_path = PathJoinSubstitution([
        FindPackageShare('ros_arduino_bridge'),
        'deployment', 'shared', 'robot_description.xacro'
    ])

    # Robot State Publisher - publishes TF tree
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path]),
            'use_sim_time': False,
            'publish_frequency': 30.0,
        }]
    )

    # EKF config path
    ekf_config_path = PathJoinSubstitution([
        pkg_share,
        'config', 'ekf_config.yaml'
    ])

    # Arduino Bridge - core robot interface
    # When using EKF: publish_tf is disabled (EKF will handle TF)
    # When not using EKF: publish_tf is enabled (bridge handles TF)
    arduino_bridge = Node(
        package='ros_arduino_bridge',
        executable='ros_arduino_bridge',
        name='ros_arduino_bridge',
        output='screen',
        parameters=[{
            # Hardware configuration
            'serial_port': arduino_port,
            'baud_rate': 57600,
            
            # Calibrated robot parameters
            'base_width': 0.249000,      # Effective track width (20.8cm + 4.1cm wheel width)
            'wheel_radius': 0.042500,    # 85mm diameter wheels
            'encoder_ticks_per_rev': 373, # Measured encoder ticks
            
            # Frame names
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            
            # TF publishing control (conditional based on use_ekf)
            # If using EKF, disable bridge TF (EKF will publish it)
            # If not using EKF, enable bridge TF
            # NotSubstitution inverts the boolean value
            'publish_tf': NotSubstitution(use_ekf),
        }]
    )
    
    # EKF Node - sensor fusion (only launched when use_ekf=true)
    # Fuses wheel encoders (/odom) + IMU (/imu/data) → /odometry/filtered
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path],
        condition=IfCondition(use_ekf)
    )

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('arduino_port', 
                             default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
                             description='Arduino serial port (device ID)'),
        DeclareLaunchArgument('use_ekf',
                             default_value='false',
                             description='Enable EKF sensor fusion (true/false)'),
        
        # Nodes
        robot_state_publisher,
        arduino_bridge,
        ekf_node,  # Only launches if use_ekf=true
        # NO LIDAR - Launch separately
    ])