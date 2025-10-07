#!/usr/bin/env python3
"""
Launch file for disease detection node on Laptop
Subscribes to camera images from Raspberry Pi over network and performs
potato disease classification using PyTorch model
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
    
    # Path to parameters file
    params_file = os.path.join(
        pkg_share, 'config', 'laptop', 'disease_detection_params.yaml'
    )
    
    # Declare launch arguments
    use_compressed_arg = DeclareLaunchArgument(
        'use_compressed',
        default_value='true',
        description='Subscribe to compressed images (recommended for network)'
    )
    
    display_arg = DeclareLaunchArgument(
        'display',
        default_value='true',
        description='Display annotated images in OpenCV window'
    )
    
    publish_annotated_arg = DeclareLaunchArgument(
        'publish_annotated',
        default_value='true',
        description='Publish annotated images to /disease_detection/annotated_image'
    )
    
    inference_rate_arg = DeclareLaunchArgument(
        'inference_rate',
        default_value='1.0',
        description='Inference frequency in Hz (lower = less CPU, higher = more detections)'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.0',
        description='Minimum confidence threshold to report detections (0.0-1.0)'
    )
    
    # Disease Detection Node
    disease_detection_node = Node(
        package='rpi_camera_package',
        executable='disease_detection_node',
        name='disease_detection_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            params_file,
            {
                'use_compressed': LaunchConfiguration('use_compressed'),
                'display_output': LaunchConfiguration('display'),
                'publish_annotated': LaunchConfiguration('publish_annotated'),
                'inference_rate': LaunchConfiguration('inference_rate'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
            }
        ]
    )
    
    return LaunchDescription([
        use_compressed_arg,
        display_arg,
        publish_annotated_arg,
        inference_rate_arg,
        confidence_threshold_arg,
        disease_detection_node,
    ])
