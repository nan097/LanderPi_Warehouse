"""Start the original controller board and selected hardware interfaces.

External vendor ROS drivers must be installed separately.
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(package, relative_path):
    path = Path(get_package_share_directory(package)) / relative_path
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)))


def _setup(context):
    os.environ.setdefault('need_compile', 'True')
    os.environ.setdefault('MACHINE_TYPE', 'mecanum')
    actions = [_include('ros_robot_controller', 'launch/ros_robot_controller.launch.py')]

    lidar = LaunchConfiguration('lidar').perform(context).lower()
    lidar_files = {
        'ld19': 'ldlidar_LD19.launch.py',
        'ms200': 'ms200_scan.launch.py',
        'a1': 'sllidar_a1.launch.py',
        'g4': 'ydlidar_g4.launch.py',
    }
    if lidar in lidar_files:
        actions.append(_include('peripherals', 'launch/include/' + lidar_files[lidar]))
    elif lidar != 'none':
        raise ValueError('lidar must be none, ld19, ms200, a1, or g4')

    camera = LaunchConfiguration('camera').perform(context).lower()
    camera_files = {
        'usb': 'launch/usb_cam.launch.py',
        'ascamera': 'launch/include/ascamera.launch.py',
        'aurora': 'launch/include/aurora930.launch.py',
    }
    if camera in camera_files:
        actions.append(_include('peripherals', camera_files[camera]))
    elif camera != 'none':
        raise ValueError('camera must be none, usb, ascamera, or aurora')

    if LaunchConfiguration('joystick').perform(context).lower() == 'true':
        actions.append(_include('peripherals', 'launch/joystick_control.launch.py'))
    if LaunchConfiguration('mic').perform(context).lower() == 'true':
        actions.append(Node(
            package='xf_mic_asr_offline',
            executable='awake_node.py',
            output='screen',
            parameters=[{'port': '/dev/ring_mic', 'mic_type': 'mic6_circle'}],
        ))
    if LaunchConfiguration('imu_filter').perform(context).lower() == 'true':
        actions.append(_include('peripherals', 'launch/imu_filter.launch.py'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='none'),
        DeclareLaunchArgument('camera', default_value='none'),
        DeclareLaunchArgument('joystick', default_value='false'),
        DeclareLaunchArgument('mic', default_value='false'),
        DeclareLaunchArgument('imu_filter', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
