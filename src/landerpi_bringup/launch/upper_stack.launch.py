"""Foundation launch: explicit dry-run; no hardware launches implicitly."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('execution_mode', default_value='dry_run'),
        Node(package='landerpi_robot_monitor', executable='robot_monitor_node', output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('landerpi_task_manager'), 'launch', 'task_manager.launch.py'])),
            launch_arguments={'execution_mode': LaunchConfiguration('execution_mode')}.items()),
    ])
