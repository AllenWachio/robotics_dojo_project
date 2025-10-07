#!/usr/bin/env python3
"""
Pi Competition Hardware Launch File
Runs on Raspberry Pi - handles ALL hardware
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch arguments
    arduino_port = LaunchConfiguration('arduino_port')
    lidar_port = LaunchConfiguration('lidar_port')
    
    # Robot description
    urdf_path = PathJoinSubstitution([
        FindPackageShare('ros_arduino_bridge'),
        'urdf', 'robot.urdf.xacro'
    ])

    # Robot State Publisher
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

    # Arduino Bridge (motors, encoders, IMU, RGB sensor, servos)
    arduino_bridge = Node(
        package='ros_arduino_bridge',
        executable='ros_arduino_bridge',
        name='ros_arduino_bridge',
        output='screen',
        parameters=[{
            'serial_port': arduino_port,
            'baud_rate': 57600,
            'base_width': 0.249000,
            'wheel_radius': 0.042500,
            'encoder_ticks_per_rev': 373,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
        }]
    )

    # LiDAR
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': lidar_port,
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Sensitivity',
            'auto_reconnect': True,
        }],
        output='screen'
    )

    # Pi Camera (compressed for network efficiency)
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera_node',
        output='screen',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [640, 480],
            'camera_frame_id': 'camera_link',
            'time_per_frame': [1, 30],  # 30 FPS
        }]
    )

    # Image compression for network transmission
    image_compressor = Node(
        package='image_transport',
        executable='republish',
        name='image_compressor',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/image_raw'),
            ('out/compressed', '/camera/image_raw/compressed')
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('arduino_port', 
            default_value='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'),
        DeclareLaunchArgument('lidar_port',
            default_value='/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'),
        
        robot_state_publisher,
        arduino_bridge,
        lidar_node,
        camera_node,
        image_compressor,
    ])
