from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    RASPBERRY PI ROBOT-SIDE LAUNCH
    
    This launch file runs on the Raspberry Pi and handles:
    - Arduino communication (odometry, motor control)
    - Robot state publishing (TF tree)
    - Hardware interfaces only
    
    All processing and visualization handled by laptop base station
    Publishes topics that laptop automatically discovers over network
    """
    
    # Launch arguments - using device ID for consistent device identification
    arduino_port = LaunchConfiguration('arduino_port', default='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0')
    
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
            'publish_frequency': 30.0,  # Efficient for network
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
            'base_width': 0.249000,      # Effective track width (20.8cm + 4.1cm wheel width)
            'wheel_radius': 0.042500,    # 85mm diameter wheels
            'encoder_ticks_per_rev': 373, # Measured encoder ticks
            
            # Motion limits
            #'max_linear_speed': 0.5,     # m/s
            #'max_angular_speed': 1.0,    # rad/s
            
            # Frame names
            'odom_frame': 'odom',
            'base_frame': 'base_link',
        }]
    )

    # LiDAR Driver - CRITICAL for SLAM
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',  # Device ID approach (more reliable than ttyUSB)
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Sensitivity', 
            'auto_reconnect': True,   # Added for better reliability
        }],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('arduino_port', default_value=arduino_port,
                             description='Arduino serial port (device ID)'),
        
        robot_state_publisher,
        arduino_bridge,
        lidar_node,  # ESSENTIAL for SLAM to work
    ])