#!/usr/bin/env python3
"""
Launch file for color detection node on Laptop
Subscribes to camera images from Raspberry Pi over network
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    # Declare launch arguments
    use_compressed_arg = DeclareLaunchArgument(
        'use_compressed',
        default_value='true',
        description='Subscribe to compressed images (recommended for network)'
    )
    
    display_arg = DeclareLaunchArgument(
        'display',
        default_value='true',
        description='Display processed images in OpenCV window'
    )
    
    min_area_arg = DeclareLaunchArgument(
        'min_area',
        default_value='1000',
        description='Minimum contour area in pixels to detect'
    )
    
    publish_processed_arg = DeclareLaunchArgument(
        'publish_processed',
        default_value='true',
        description='Publish processed images with detections'
    )
    
    # Color Detection Node
    color_detection_node = Node(
        package='rpi_camera_package',
        executable='color_detection_node',
        name='color_detection_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_compressed': LaunchConfiguration('use_compressed'),
            'display_output': LaunchConfiguration('display'),
            'min_contour_area': LaunchConfiguration('min_area'),
            'publish_processed': LaunchConfiguration('publish_processed')
        }]
    )
    
    return LaunchDescription([
        use_compressed_arg,
        display_arg,
        min_area_arg,
        publish_processed_arg,
        color_detection_node,
    ])
