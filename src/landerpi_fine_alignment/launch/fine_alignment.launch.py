from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='landerpi_fine_alignment', executable='fine_alignment_server', output='screen'),
    ])
