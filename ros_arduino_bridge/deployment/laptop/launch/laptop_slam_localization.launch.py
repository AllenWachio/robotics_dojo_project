from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition

def generate_launch_description():
    """
    LAPTOP SLAM TOOLBOX LOCALIZATION - Localize against saved serialized map
    
    This launch file runs on your laptop and provides:
    - SLAM Toolbox in LOCALIZATION mode (not mapping)
    - Loads previously saved serialized maps (.data + .posegraph)
    - Provides map->odom transform for navigation
    - RViz visualization with localization tools
    
    This is an ALTERNATIVE to AMCL for localization.
    Use this when you have serialized SLAM Toolbox maps.
    
    Prerequisites: 
    1. Robot must be running (Pi hardware interface)
    2. Serialized map must exist (.data + .posegraph files)
    3. You must provide initial pose estimate in RViz (2D Pose Estimate)
    """
    
    # Launch arguments
    map_file = LaunchConfiguration('map_file', default='robot_map')
    map_path = LaunchConfiguration('map_path', default='~/ros2_ws/maps/')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Build full map path (without extension - SLAM Toolbox adds .data/.posegraph)
    # Note: PathJoinSubstitution doesn't work well with ~ so we use full path
    map_file_base = PathJoinSubstitution([
        map_path, map_file
    ])
    
    # Configuration file paths
    slam_localization_params_file = PathJoinSubstitution([
        FindPackageShare('ros_arduino_bridge'),
        'deployment', 'laptop', 'config', 'slam_localization_laptop.yaml'
    ])
    
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('ros_arduino_bridge'),
        'deployment', 'laptop', 'config', 'laptop_rviz_config.rviz'
    ])

    # SLAM Toolbox - Localization mode
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',  # Use localization node, not async!
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_localization_params_file,
            {
                'use_sim_time': use_sim_time,
                'map_file_name': map_file_base,  # Override config file parameter
            }
        ]
    )

    # RViz2 for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz)
    )

    # Robot diagnostics
    diagnostics_node = Node(
        package='ros_arduino_bridge',
        executable='slam_diagnostics',
        name='slam_diagnostics',
        output='screen',
        parameters=[{
            'check_localization': True,
        }]
    )

    # Delayed startup for proper initialization
    delayed_rviz = TimerAction(period=3.0, actions=[rviz_node])
    delayed_diagnostics = TimerAction(period=5.0, actions=[diagnostics_node])

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('map_file', default_value=map_file,
                             description='Map file base name (without extension, e.g., my_map)'),
        DeclareLaunchArgument('map_path', default_value=map_path,
                             description='Directory path where maps are stored'),
        DeclareLaunchArgument('use_rviz', default_value=use_rviz,
                             description='Launch RViz2 for visualization'),
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                             description='Use simulation time'),
        
        # Core nodes
        slam_toolbox_node,
        
        # UI and monitoring (delayed start)
        delayed_rviz,
        delayed_diagnostics,
    ])
