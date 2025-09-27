from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # Path to the sllidar launch file
    sllidar_launch_path = PathJoinSubstitution([
        FindPackageShare('sllidar_ros2'),
        'launch',
        'sllidar_a1_launch.py'
    ])

    # Path to the slam toolbox configuration
    slam_params_file = PathJoinSubstitution([
        FindPackageShare('slam_toolbox'),
        'config',
        'mapper_params_online_async.yaml'
    ])

    # Declare launch arguments
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    # Include the sllidar launch file
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(sllidar_launch_path)
    )

    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': False}]
    )

    # RViz configuration for SLAM
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('slam_toolbox'),
        'rviz',
        'slam_toolbox.rviz'
    ])

    # RViz node
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Declare the use_rviz launch argument
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value=use_rviz,
        description='Whether to start RViz'
    )

    # Create the launch description and return it
    return LaunchDescription([
        declare_use_rviz_cmd,
        sllidar_launch,
        slam_toolbox_node,
        rviz_node
    ])
