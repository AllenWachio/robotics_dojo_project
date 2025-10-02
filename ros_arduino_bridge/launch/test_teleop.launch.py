from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import FindExecutable

def generate_launch_description():
    # Arduino Bridge Node
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
            'encoder_ticks_per_rev': 283
        }]
    )
    
    # Robot State Publisher for your URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': 
                '<?xml version="1.0" ?>'
                '<robot name="new_robot_urdf">'
                '<!-- Your URDF content will be loaded from file -->'
                '</robot>',
            'use_sim_time': False
        }]
    )
    
    # Teleop keyboard (start after a delay to ensure bridge is ready)
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e'  # Opens in new terminal window
    )
    
    # Delay teleop start until after bridge is running
    delayed_teleop = TimerAction(
        period=3.0,  # Wait 3 seconds
        actions=[teleop_node]
    )
    
    return LaunchDescription([
        arduino_bridge,
        robot_state_publisher,
        delayed_teleop,
    ])
