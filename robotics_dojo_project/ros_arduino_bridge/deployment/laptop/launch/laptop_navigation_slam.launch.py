from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    """
    LAPTOP NAVIGATION WITH SLAM TOOLBOX LOCALIZATION
    
    This launch file runs on your laptop and provides:
    - SLAM Toolbox localization (instead of AMCL)
    - Complete Nav2 navigation stack
    - Path planning and obstacle avoidance
    - Goal setting and execution
    - RViz with navigation interface
    
    This is an ALTERNATIVE to the AMCL-based navigation.
    Use this when you have serialized SLAM Toolbox maps and want
    to continue using SLAM Toolbox for localization.
    
    Prerequisites: 
    1. Robot must be running (Pi hardware interface)
    2. Serialized map must exist (.data + .posegraph files)
    3. SLAM Toolbox localization must be running (or will be launched here)
    """
    
    # Launch arguments
    map_file = LaunchConfiguration('map_file', default='robot_map')
    map_path = LaunchConfiguration('map_path', default='~/ros2_ws/maps/')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    autostart = LaunchConfiguration('autostart', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    run_localization = LaunchConfiguration('run_localization', default='true')
    
    # Expand map path
    map_path_expanded = PathJoinSubstitution([
        map_path
    ])
    
    # Configuration file paths
    nav2_params_file = PathJoinSubstitution([
        FindPackageShare('ros_arduino_bridge'),
        'deployment', 'laptop', 'config', 'nav2_params.yaml'
    ])
    
    slam_localization_params_file = PathJoinSubstitution([
        FindPackageShare('ros_arduino_bridge'),
        'deployment', 'laptop', 'config', 'slam_localization_laptop.yaml'
    ])
    
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('ros_arduino_bridge'),
        'deployment', 'laptop', 'config', 'navigation_rviz_config.rviz'
    ])
    
    # Build full map path (without extension)
    map_file_base = PathJoinSubstitution([
        map_path, map_file
    ])

    # Rewrite the nav2 params file to include dynamic values
    param_substitutions = {
        'use_sim_time': use_sim_time,
    }
    
    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # SLAM Toolbox Localization - provides map->odom transform
    # This REPLACES AMCL
    slam_localization_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_localization_params_file,
            {
                'use_sim_time': use_sim_time,
                'map_file_name': map_file_base,
            }
        ],
        condition=IfCondition(run_localization)
    )

    # Nav2 Controller Server - path following
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params]
    )

    # Nav2 Planner Server - path planning
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params]
    )

    # Nav2 Behavior Server - recovery behaviors
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params]
    )

    # Nav2 BT Navigator - behavior tree navigation
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params]
    )

    # Nav2 Waypoint Follower - sequential goal execution
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params]
    )

    # Nav2 Velocity Smoother - smooth velocity commands
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[configured_params]
    )

    # Nav2 Lifecycle Manager - manages node lifecycle
    # NOTE: This does NOT include map_server or amcl since we're using SLAM Toolbox
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'controller_server',
                'planner_server', 
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother'
            ]
        }]
    )

    # RViz2 with navigation interface
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_navigation',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz)
    )

    # Robot diagnostics for navigation
    nav_diagnostics_node = Node(
        package='ros_arduino_bridge',
        executable='slam_diagnostics',
        name='nav_diagnostics',
        output='screen',
        parameters=[{
            'check_navigation': True,
            'check_localization': True,
            'check_path_planning': True
        }]
    )

    # Delayed startup for proper initialization
    delayed_rviz = TimerAction(period=3.0, actions=[rviz_node])
    delayed_diagnostics = TimerAction(period=5.0, actions=[nav_diagnostics_node])

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('map_file', default_value=map_file,
                             description='Map file base name (without extension)'),
        DeclareLaunchArgument('map_path', default_value=map_path,
                             description='Directory path where maps are stored'),
        DeclareLaunchArgument('use_rviz', default_value=use_rviz,
                             description='Launch RViz2 with navigation interface'),
        DeclareLaunchArgument('autostart', default_value=autostart,
                             description='Automatically start navigation nodes'),
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                             description='Use simulation time'),
        DeclareLaunchArgument('run_localization', default_value=run_localization,
                             description='Run SLAM Toolbox localization (set false if already running)'),
        
        # Localization (replaces AMCL)
        slam_localization_node,
        
        # Core navigation nodes
        controller_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        velocity_smoother_node,
        lifecycle_manager_node,
        
        # UI and monitoring (delayed start)
        delayed_rviz,
        delayed_diagnostics,
    ])
