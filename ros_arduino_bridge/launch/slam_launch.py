from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to the SLAM Toolbox configuration
    slam_params_file = PathJoinSubstitution([
        FindPackageShare('ros_arduino_bridge'),
        'config',
        'mapper_params_online_async.yaml'
    ])

    # Path to robot URDF
    urdf_path = PathJoinSubstitution([
        FindPackageShare('ros_arduino_bridge'),
        'urdf',
        'new_robot_urdf.xacro'
    ])

    # Robot State Publisher (needed for TF tree)
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

    # Arduino Bridge (for odometry and tf)
    arduino_bridge = Node(
        package='ros_arduino_bridge',
        executable='ros_arduino_bridge',
        name='ros_arduino_bridge',
        output='screen',
        parameters=[{
            'serial_port': '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0',
            'baud_rate': 57600,
            'base_width': 0.208000,
            'wheel_radius': 0.042500,
            'encoder_ticks_per_rev': 44
        }]
    )

    # SLAM Toolbox node (no scan throttling needed - SLAM can handle full rate)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': False}]
    )

    # Return the launch description
    return LaunchDescription([
        robot_state_publisher,
        arduino_bridge,
        slam_toolbox_node
    ])
