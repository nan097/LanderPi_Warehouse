from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value='true') for name in ('gui', 'rviz', 'moveit')],
        DeclareLaunchArgument('demo', default_value='false'),
        DeclareLaunchArgument('gazebo_port', default_value='0'),
        DeclareLaunchArgument('ros_domain_id', default_value='42'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('landerpi_simulation'), 'launch', 'simulation.launch.py'])),
            launch_arguments={name: LaunchConfiguration(name) for name in ('gui', 'rviz', 'moveit', 'demo', 'gazebo_port', 'ros_domain_id')}.items()),
    ])
