from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Launch arguments with defaults - using device ID for Arduino
    serial_port = LaunchConfiguration('serial_port', default='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    urdf_file = LaunchConfiguration('urdf_file', default='new_robot_urdf.xacro')
    publish_tf = LaunchConfiguration('publish_tf', default='true')  # Default: True for standalone use
    
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
    
    # ROS Arduino Bridge Node (handles joint states internally)
    arduino_bridge = Node(
        package='ros_arduino_bridge',
        executable='ros_arduino_bridge',
        name='ros_arduino_bridge',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': 57600,
            'base_width': 0.249000,      # Effective track width (20.8cm + 4.1cm wheel width)
            'wheel_radius': 0.042500,    # 85mm diameter wheels
            'encoder_ticks_per_rev': 447,  # Calibrated value
            'max_linear_speed': 0.5,
            'max_angular_speed': 1.0,
            'publish_tf': publish_tf     # Configurable: True for standalone, False with EKF
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
                             description='Serial port for Arduino (device ID)'),
        DeclareLaunchArgument('use_rviz', default_value=use_rviz,
                             description='Launch RViz2 for visualization'),
        DeclareLaunchArgument('urdf_file', default_value=urdf_file,
                             description='URDF/XACRO file name'),
        DeclareLaunchArgument('publish_tf', default_value=publish_tf,
                             description='Publish odom→base_link TF (set false when using EKF)'),
        
        robot_state_publisher,
        arduino_bridge,
        rviz_node,
    ])
