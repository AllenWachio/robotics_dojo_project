from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_arduino_bridge',
            executable='ros_arduino_bridge',
            name='ros_arduino_bridge',
            output='screen',
        ),
    ])
