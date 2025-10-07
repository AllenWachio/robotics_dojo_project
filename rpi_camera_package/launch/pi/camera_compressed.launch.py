#!/usr/bin/env python3
"""
Launch file for V4L2 camera with image compression on Raspberry Pi
Publishes compressed images for efficient network transmission
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('rpi_camera_package')
    
    # Path to camera parameters file
    camera_params_file = os.path.join(pkg_share, 'config', 'pi', 'camera_params.yaml')
    
    # Declare launch arguments
    video_device_arg = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='Video device path'
    )
    
    jpeg_quality_arg = DeclareLaunchArgument(
        'jpeg_quality',
        default_value='80',
        description='JPEG compression quality (0-100, higher is better quality but larger size)'
    )
    
    # V4L2 Camera Node
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        namespace='camera',
        parameters=[camera_params_file, {
            'video_device': LaunchConfiguration('video_device')
        }],
        remappings=[
            ('/camera/camera/image_raw', '/camera/image_raw'),
            ('/camera/camera/camera_info', '/camera/camera_info')
        ],
        output='screen',
        emulate_tty=True
    )
    
    # Image Transport Republisher for compression
    # This node subscribes to raw images and republishes as compressed
    republish_node = Node(
        package='image_transport',
        executable='republish',
        name='image_republisher',
        arguments=[
            'raw',
            'compressed',
        ],
        remappings=[
            ('in', '/camera/image_raw'),
            ('out', '/camera/image_raw'),
        ],
        parameters=[{
            'compressed.format': 'jpeg',
            'compressed.jpeg_quality': LaunchConfiguration('jpeg_quality'),
            'compressed.png_level': 9
        }],
        output='screen'
    )
    
    return LaunchDescription([
        video_device_arg,
        jpeg_quality_arg,
        v4l2_camera_node,
        republish_node,
    ])
