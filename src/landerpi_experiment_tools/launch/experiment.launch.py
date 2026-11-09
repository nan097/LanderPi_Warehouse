from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='landerpi_experiment_tools', executable='benchmark_runner', output='screen'),
    ])
