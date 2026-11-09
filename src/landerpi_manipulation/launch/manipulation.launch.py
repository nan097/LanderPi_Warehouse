from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='landerpi_manipulation', executable='manipulation_server', output='screen'),
        Node(package='landerpi_manipulation', executable='planning_scene_bridge', output='screen'),
    ])
