"""Complete simulation: ideal odometry localization, Nav2, RGB-D and manipulation."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def stack(context):
    share = Path(get_package_share_directory('landerpi_simulation'))
    config = str(share / 'config/nav2.yaml')
    navshare = Path(get_package_share_directory('nav2_bt_navigator'))
    names = ['map_server', 'planner_server', 'controller_server', 'behavior_server', 'bt_navigator']
    nodes = [Node(package='nav2_map_server', executable='map_server', name='map_server', parameters=[config, {'yaml_filename': str(share / 'config/sorting_map.yaml'), 'use_sim_time': True}], output='screen')]
    for (name, pkg, exe) in [('planner_server', 'nav2_planner', 'planner_server'), ('controller_server', 'nav2_controller', 'controller_server'), ('behavior_server', 'nav2_behaviors', 'behavior_server'), ('bt_navigator', 'nav2_bt_navigator', 'bt_navigator')]:
        extra = {'default_nav_to_pose_bt_xml': str(navshare / 'behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml')} if name == 'bt_navigator' else {}
        nodes.append(Node(package=pkg, executable=exe, name=name, parameters=[config, extra], output='screen'))
    nodes.append(TimerAction(period=10.0, actions=[Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation', parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': names, 'bond_timeout': 0.0}], output='screen')]))
    nodes.append(Node(package='landerpi_simulation', executable='sort_runtime.py', parameters=[{'use_sim_time': True, 'csv_path': LaunchConfiguration('csv_path').perform(context)}], output='screen'))
    nodes.append(Node(package='landerpi_simulation', executable='sort_demo.py', output='screen', condition=IfCondition(LaunchConfiguration('autorun'))))
    return nodes

def generate_launch_description():
    share = Path(get_package_share_directory('landerpi_simulation'))
    return LaunchDescription([DeclareLaunchArgument('autorun', default_value='false'), DeclareLaunchArgument('gui', default_value='true'), DeclareLaunchArgument('rviz', default_value='true'), DeclareLaunchArgument('ros_domain_id', default_value='75'), DeclareLaunchArgument('gazebo_port', default_value='0'), DeclareLaunchArgument('csv_path', default_value='/tmp/landerpi_experiments/simulation_tasks_v2.csv'), SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')), IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/simulation.launch.py')), launch_arguments={**{k: LaunchConfiguration(k) for k in ['gui', 'rviz', 'ros_domain_id', 'gazebo_port']}, 'rviz_config': str(share / 'rviz/sort.rviz')}.items()), OpaqueFunction(function=stack)])
