from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Launch arguments with defaults
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    urdf_file = LaunchConfiguration('urdf_file', default='new_robot_urdf.xacro')
    
    # Paths to package resources
    urdf_path = PathJoinSubstitution([
        FindPackageShare('ros_arduino_bridge'),
        'urdf',
        urdf_file
    ])
    
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('ros_arduino_bridge'),
        'config',
        'view_robot.rviz'
    ])
    
    # Robot State Publisher Node
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
    
    # ROS Arduino Bridge Node
    arduino_bridge = Node(
        package='ros_arduino_bridge',
        executable='ros_arduino_bridge',
        name='ros_arduino_bridge',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': 57600,
            'base_width': 0.215446,
            'wheel_radius': 0.085,
            'encoder_ticks_per_rev': 44
        }]
    )
    
    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value=serial_port,
                             description='Serial port for Arduino'),
        DeclareLaunchArgument('use_rviz', default_value=use_rviz,
                             description='Launch RViz2 for visualization'),
        DeclareLaunchArgument('urdf_file', default_value=urdf_file,
                             description='URDF/XACRO file name'),
        
        robot_state_publisher,
        arduino_bridge,
        rviz_node,
    ])
