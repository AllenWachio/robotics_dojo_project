#!/usr/bin/env python3
"""
Laptop Competition Processing Launch File
Runs on Laptop - handles Nav2, ML, vision processing
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml
import os


def generate_launch_description():
    # Launch arguments
    map_file = LaunchConfiguration('map_file')
    map_path = LaunchConfiguration('map_path')
    use_rviz = LaunchConfiguration('use_rviz')
    use_disease_detection = LaunchConfiguration('use_disease_detection')
    use_color_detection = LaunchConfiguration('use_color_detection')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Package paths
    pkg_share = FindPackageShare('ros_arduino_bridge')
    camera_pkg_share = FindPackageShare('rpi_camera_package')
    
    # Config file paths
    nav2_params_file = PathJoinSubstitution([
        pkg_share, 'config', 'nav2_params.yaml'
    ])
    
    rviz_config_path = PathJoinSubstitution([
        pkg_share, 'config', 'navigation_rviz_config.rviz'
    ])
    
    disease_params_file = PathJoinSubstitution([
        camera_pkg_share, 'config', 'laptop', 'disease_detection_params.yaml'
    ])
    
    # Map file path
    map_file_path = PathJoinSubstitution([map_path, map_file])
    
    # Rewrite nav2 params with dynamic values
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_file_path
    }
    
    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # ========================================
    # NAV2 NAVIGATION STACK
    # ========================================
    
    # Map Server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_file_path
        }]
    )

    # AMCL Localization
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_params]
    )

    # Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params]
    )

    # Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params]
    )

    # Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params]
    )

    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params]
    )

    # Waypoint Follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params]
    )

    # Lifecycle Manager (starts/manages Nav2 nodes)
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'map_server',
                'amcl',
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower'
            ]
        }]
    )

    # ========================================
    # VISION PROCESSING
    # ========================================
    
    # Disease Detection (ML model)
    disease_detection = Node(
        package='rpi_camera_package',
        executable='disease_detection_node',
        name='disease_detection_node',
        output='screen',
        condition=IfCondition(use_disease_detection),
        parameters=[
            disease_params_file,
            {
                'use_compressed': True,  # Subscribe to compressed images
                'display_output': False,  # No GUI on laptop
                'inference_rate': 1.0,
                'confidence_threshold': 0.0,
            }
        ]
    )

    # Color Detection
    color_detection = Node(
        package='rpi_camera_package',
        executable='color_detection_node',
        name='color_detection_node',
        output='screen',
        condition=IfCondition(use_color_detection),
        parameters=[{
            'use_compressed': True,
            'display_output': False,
            'min_contour_area': 1000,
            'publish_processed': True
        }]
    )

    # ========================================
    # RVIZ VISUALIZATION
    # ========================================
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('map_file', default_value='gamefield.yaml'),
        DeclareLaunchArgument('map_path', default_value='~/ros2_ws/maps'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_disease_detection', default_value='true'),
        DeclareLaunchArgument('use_color_detection', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        # Nav2 nodes
        map_server,
        amcl,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
        
        # Vision processing
        disease_detection,
        color_detection,
        
        # Visualization
        rviz,
    ])
