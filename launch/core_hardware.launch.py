"""Minimal ROS 2 launch entry for the original LanderPi controller board."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    controller_launch = Path(get_package_share_directory('ros_robot_controller')) / 'launch' / 'ros_robot_controller.launch.py'
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(controller_launch))),
    ])
