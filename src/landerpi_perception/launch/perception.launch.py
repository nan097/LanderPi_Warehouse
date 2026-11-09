from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='landerpi_perception', executable='perception_server', output='screen'),
    ])
