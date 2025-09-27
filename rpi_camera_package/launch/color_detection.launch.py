from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rpi_camera_package',
            executable='color_detection_subscriber',
            name='color_detection_subscriber',
            output='screen'
        ),
    ])
