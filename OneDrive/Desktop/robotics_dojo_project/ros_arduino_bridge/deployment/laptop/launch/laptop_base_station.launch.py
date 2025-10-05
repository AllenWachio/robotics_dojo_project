from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    """
    LAPTOP BASE STATION - Complete Processing & Visualization
    
    This launch file runs on your laptop and handles:
    - SLAM processing (computationally intensive)
    - RViz visualization
    - Map saving/loading
    - Robot diagnostics
    - Remote teleop control
    
    Automatically connects to robot topics published by Raspberry Pi
    """
    
    # Launch arguments
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    save_maps = LaunchConfiguration('save_maps', default='true')
    enable_teleop = LaunchConfiguration('enable_teleop', default='true')
    enable_diagnostics = LaunchConfiguration('enable_diagnostics', default='true')
    
    # Configuration file paths
    slam_params_file = PathJoinSubstitution([
        FindPackageShare('ros_arduino_bridge'),
        'deployment', 'laptop', 'config', 'slam_config_laptop.yaml'
    ])
    
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('ros_arduino_bridge'),
        'deployment', 'laptop', 'config', 'laptop_rviz_config.rviz'
    ])

    # SLAM Toolbox - Full processing power on laptop
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {
            'use_sim_time': False,
            'use_map_saver': save_maps,
        }]
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

    # Remote teleop control
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',
        condition=IfCondition(enable_teleop)
    )

    # Robot health diagnostics
    diagnostics_node = Node(
        package='ros_arduino_bridge',
        executable='slam_diagnostics',
        name='slam_diagnostics',
        output='screen',
        condition=IfCondition(enable_diagnostics)
    )

    # Map server for saving/loading
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            # Note: Maps are now saved to ~/ros2_ws/maps/ by user scripts
            # This launch file is for mapping mode, not navigation mode
            'yaml_filename': '/tmp/placeholder_map.yaml'
        }],
        condition=IfCondition(save_maps)
    )

    # Timed startup to ensure proper initialization
    delayed_rviz = TimerAction(period=2.0, actions=[rviz_node])
    delayed_teleop = TimerAction(period=3.0, actions=[teleop_node])
    delayed_diagnostics = TimerAction(period=5.0, actions=[diagnostics_node])

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('use_rviz', default_value=use_rviz,
                             description='Launch RViz2 for visualization'),
        DeclareLaunchArgument('save_maps', default_value=save_maps,
                             description='Enable map saving functionality'),
        DeclareLaunchArgument('enable_teleop', default_value=enable_teleop,
                             description='Enable remote teleop control'),
        DeclareLaunchArgument('enable_diagnostics', default_value=enable_diagnostics,
                             description='Enable robot diagnostics monitoring'),
        
        # Core nodes
        slam_toolbox_node,
        map_server_node,
        
        # UI nodes (delayed start)
        delayed_rviz,
        delayed_teleop,
        delayed_diagnostics,
    ])