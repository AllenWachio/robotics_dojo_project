from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Camera Publisher Node
        Node(
            package='rpi_camera_package',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen',
            parameters=[{
                'frame_rate': 30,
                'resolution': [640, 480]
            }]
        ),
        
        # Color Detection Node
        Node(
            package='rpi_camera_package',
            executable='color_detection_subscriber',
            name='color_detection_subscriber',
            output='screen'
        ),
        
        # Potato Disease Detection Node (from external package)
        Node(
            package='rdj2025_potato_disease_detection',
            executable='potato_disease_detection_node',
            name='potato_disease_detector',
            output='screen',
            remappings=[
                ('/image', '/camera/image_raw')
            ]
        ),
    ])
