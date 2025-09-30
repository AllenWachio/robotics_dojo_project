from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    RASPBERRY PI ARDUINO-ONLY LAUNCH
    
    This launch file runs only:
    - Arduino communication (odometry, motor control)
    - Robot state publishing (TF tree)
    
    LiDAR is launched separately to avoid conflicts
    """
    
    # Launch arguments
    arduino_port = LaunchConfiguration('arduino_port', default='/dev/ttyUSB0')
    
    # Robot description path
    urdf_path = PathJoinSubstitution([
        FindPackageShare('ros_arduino_bridge'),
        'deployment', 'shared', 'robot_description.xacro'
    ])

    # Robot State Publisher - publishes TF tree
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path]),
            'use_sim_time': False,
            'publish_frequency': 30.0,
        }]
    )

    # Arduino Bridge - core robot interface
    arduino_bridge = Node(
        package='ros_arduino_bridge',
        executable='ros_arduino_bridge',
        name='ros_arduino_bridge',
        output='screen',
        parameters=[{
            # Hardware configuration
            'serial_port': arduino_port,
            'baud_rate': 57600,
            
            # Calibrated robot parameters
            'base_width': 0.208000,      # 20.8cm wheelbase
            'wheel_radius': 0.042500,    # 85mm diameter wheels
            'encoder_ticks_per_rev': 44, # 25GA370 motor specs
            
            # Frame names
            'odom_frame': 'odom',
            'base_frame': 'base_link',
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('arduino_port', default_value=arduino_port,
                             description='Arduino serial port'),
        
        robot_state_publisher,
        arduino_bridge,
        # NO LIDAR - Launch separately
    ])