"""Stationary RGB-D/MTC grasp demo using the navigation task simulation stack."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def setup(context):
    color = LaunchConfiguration('target_color').perform(context)
    if color not in ('red', 'green', 'blue'):
        raise ValueError('target_color must be red, green or blue')
    share = Path(get_package_share_directory('landerpi_simulation'))
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/simulation.launch.py')),
            launch_arguments={
                **{key:LaunchConfiguration(key) for key in ('gui','rviz','gazebo_port','ros_domain_id','target_color','velocity_scale','acceleration_scale','gripper_velocity_scale','gripper_acceleration_scale','place_x','place_y','place_z','observe_joint2','observe_joint3','observe_joint4')},
                'world':LaunchConfiguration('grasp_world'),
                'spawn_x':'0.4','spawn_y':'0.0','mtc_grasp':'true','moveit':'true',
                'demo':'false','onboard_camera':'true','legacy_camera':'false',
                'rviz_config':str(Path(get_package_share_directory('landerpi_mtc_grasp'))/'rviz/color_grasp.rviz')}.items()),
        Node(package='landerpi_mtc_grasp', executable='color_vision.py',
             parameters=[{'use_sim_time':True,'target_color':color}],output='screen')]


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(key,default_value=value) for key,value in dict(velocity_scale='0.7',acceleration_scale='0.65',gripper_velocity_scale='0.8',gripper_acceleration_scale='0.8',place_x='0.08',place_y='0.24',place_z='0.07',observe_joint2='0.1',observe_joint3='-0.8',observe_joint4='-2.0').items()],
        DeclareLaunchArgument('target_color',default_value='red'),
        DeclareLaunchArgument('grasp_world',default_value=str(Path(get_package_share_directory('landerpi_simulation'))/'worlds/mtc_color_grasp.world')),
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('gazebo_port',default_value='0'),
        DeclareLaunchArgument('ros_domain_id',default_value='79'),
        SetEnvironmentVariable('ROS_DOMAIN_ID',LaunchConfiguration('ros_domain_id')),
        OpaqueFunction(function=setup)])
