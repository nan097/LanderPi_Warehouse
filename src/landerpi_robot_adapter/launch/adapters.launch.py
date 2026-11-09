from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='landerpi_robot_adapter', executable='base_adapter_node', output='screen'),
        Node(package='landerpi_robot_adapter', executable='arm_trajectory_adapter_node', output='screen'),
        Node(package='landerpi_robot_adapter', executable='gripper_adapter_node', output='screen'),
        Node(package='landerpi_robot_adapter', executable='robot_state_adapter_node', output='screen'),
    ])
