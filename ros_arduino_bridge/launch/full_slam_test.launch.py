from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Launch arguments
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # Paths to configuration files
    slam_params_file = PathJoinSubstitution([
        FindPackageShare('ros_arduino_bridge'),
        'config',
        'mapper_params_online_async.yaml'
    ])
    
    urdf_path = PathJoinSubstitution([
        FindPackageShare('ros_arduino_bridge'),
        'urdf',
        'new_robot_urdf.xacro'
    ])
    
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('ros_arduino_bridge'),
        'config',
        'view_robot.rviz'
    ])

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path]),
            'use_sim_time': False
        }]
    )

    # Arduino Bridge Node
    arduino_bridge = Node(
        package='ros_arduino_bridge',
        executable='ros_arduino_bridge',
        name='ros_arduino_bridge',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': 57600,
            'base_width': 0.208000,
            'wheel_radius': 0.042500,
            'encoder_ticks_per_rev': 44,
            'max_linear_speed': 0.5,
            'max_angular_speed': 1.0
        }]
    )

    # SLAM Toolbox Node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': False}]
    )

    # Teleop Node for testing
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e'
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz)
    )

    # Delay teleop and rviz to ensure other nodes are ready
    delayed_teleop = TimerAction(
        period=3.0,
        actions=[teleop_node]
    )
    
    delayed_rviz = TimerAction(
        period=2.0,
        actions=[rviz_node]
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value=serial_port,
                             description='Serial port for Arduino'),
        DeclareLaunchArgument('use_rviz', default_value=use_rviz,
                             description='Launch RViz2 for visualization'),
        
        robot_state_publisher,
        arduino_bridge,
        slam_toolbox_node,
        delayed_rviz,
        delayed_teleop,
    ])