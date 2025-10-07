#!/usr/bin/env python3
"""
Full Processing Launch File for Laptop
Launches BOTH color detection AND disease detection nodes simultaneously
Both nodes subscribe to the same camera feed and process independently
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('rpi_camera_package')
    
    # Path to parameters files
    color_params_file = os.path.join(
        pkg_share, 'config', 'laptop', 'color_detection_params.yaml'
    ) if os.path.exists(os.path.join(
        pkg_share, 'config', 'laptop', 'color_detection_params.yaml'
    )) else None
    
    disease_params_file = os.path.join(
        pkg_share, 'config', 'laptop', 'disease_detection_params.yaml'
    )
    
    # Declare launch arguments
    use_compressed_arg = DeclareLaunchArgument(
        'use_compressed',
        default_value='true',
        description='Subscribe to compressed images (recommended for network)'
    )
    
    display_color_arg = DeclareLaunchArgument(
        'display_color',
        default_value='true',
        description='Display color detection window'
    )
    
    display_disease_arg = DeclareLaunchArgument(
        'display_disease',
        default_value='true',
        description='Display disease detection window'
    )
    
    inference_rate_arg = DeclareLaunchArgument(
        'inference_rate',
        default_value='1.0',
        description='Disease inference frequency in Hz'
    )
    
    # Color Detection Node
    color_detection_node = Node(
        package='rpi_camera_package',
        executable='color_detection_node',
        name='color_detection_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'use_compressed': LaunchConfiguration('use_compressed'),
                'display_output': LaunchConfiguration('display_color'),
                'min_contour_area': 1000,
                'publish_processed': True
            }
        ]
    )
    
    # Disease Detection Node
    disease_detection_node = Node(
        package='rpi_camera_package',
        executable='disease_detection_node',
        name='disease_detection_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            disease_params_file,
            {
                'use_compressed': LaunchConfiguration('use_compressed'),
                'display_output': LaunchConfiguration('display_disease'),
                'inference_rate': LaunchConfiguration('inference_rate'),
                'confidence_threshold': 0.0,
            }
        ]
    )
    
    return LaunchDescription([
        use_compressed_arg,
        display_color_arg,
        display_disease_arg,
        inference_rate_arg,
        color_detection_node,
        disease_detection_node,
    ])
