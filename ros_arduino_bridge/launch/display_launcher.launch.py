from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import xacro
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory
    share_dir = get_package_share_directory('ros_arduino_bridge')

    # Path to the URDF file
    xacro_file = os.path.join(share_dir, 'urdf', 'new_robot_urdf.xacro')

    # Process the xacro file to get the robot description
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # Path to the RViz configuration file
    rviz_config_file = os.path.join(share_dir, 'config', 'view_robot.rviz')

    # Declare launch arguments
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True',
        description='Flag to enable joint_state_publisher_gui'
    )
    publish_state_arg = DeclareLaunchArgument(
        name='publish_state',
        default_value='True',
        description='Flag to enable publishing the robot state'
    )

    # Launch configurations
    show_gui = LaunchConfiguration('gui')
    publish_state = LaunchConfiguration('publish_state')

    # Robot state publisher node
    robot_state_publisher_node = Node(
        condition=IfCondition(publish_state),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_urdf}]
    )

    # Joint state publisher node (non-GUI version)
    joint_state_publisher_node = Node(
        condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    # Joint state publisher GUI node
    joint_state_publisher_gui_node = Node(
        condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # Return the launch description
    return LaunchDescription([
        gui_arg,
        publish_state_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
