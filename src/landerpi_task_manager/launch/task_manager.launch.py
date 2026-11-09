from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('execution_mode', default_value='dry_run'),
        Node(package='landerpi_task_manager', executable='task_manager', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('landerpi_task_manager'),
                                               'config', 'task.yaml']),
                         {'execution_mode': LaunchConfiguration('execution_mode')}]),
    ])
