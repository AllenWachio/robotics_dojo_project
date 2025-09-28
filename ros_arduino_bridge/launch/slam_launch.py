from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to the SLAM Toolbox configuration
    slam_params_file = PathJoinSubstitution([
        FindPackageShare('ros_arduino_bridge'),
        'config',
        'mapper_params_online_async.yaml'
    ])

    # Throttle the LiDAR data
    throttle_node = Node(
        package='ros_arduino_bridge',
        executable='scan_throttle',
        name='scan_throttle',
        output='screen'
    )

    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': False}]
    )

    # Return the launch description
    return LaunchDescription([
        throttle_node,
        slam_toolbox_node
    ])
