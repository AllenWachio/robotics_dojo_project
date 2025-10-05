#!/usr/bin/env python3
"""
Alternative launch file using rpicam-vid (modern libcamera approach)
This is more reliable than v4l2_camera for Raspberry Pi Camera Module 2
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='1280',
        description='Image width'
    )
    
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='720',
        description='Image height'
    )
    
    framerate_arg = DeclareLaunchArgument(
        'framerate',
        default_value='15',
        description='Frame rate (FPS)'
    )
    
    jpeg_quality_arg = DeclareLaunchArgument(
        'jpeg_quality',
        default_value='80',
        description='JPEG compression quality (0-100)'
    )
    
    # RPi Camera Node using rpicam-vid
    camera_node = Node(
        package='rpi_camera_package',
        executable='rpicam_node',
        name='rpi_camera',
        parameters=[{
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'framerate': LaunchConfiguration('framerate'),
            'jpeg_quality': LaunchConfiguration('jpeg_quality'),
            'camera_name': 'rpi_camera'
        }],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        width_arg,
        height_arg,
        framerate_arg,
        jpeg_quality_arg,
        camera_node,
    ])
