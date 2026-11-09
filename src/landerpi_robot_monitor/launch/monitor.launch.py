from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='landerpi_robot_monitor', executable='robot_monitor_node', output='screen'),
    ])
