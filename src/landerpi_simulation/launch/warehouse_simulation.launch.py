"""Six named warehouse racks and three physical color destinations."""
from pathlib import Path
import math
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction,SetEnvironmentVariable,TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition


def stack(context):
    share=Path(get_package_share_directory('landerpi_simulation'))
    hardware = LaunchConfiguration('hardware').perform(context).lower() == 'true'
    profile=LaunchConfiguration('motion_profile').perform(context)
    if profile not in ('fast','legacy'):
        raise ValueError('motion_profile must be fast or legacy')
    # The hardware wrapper retains its original limits regardless of profile.
    if hardware: profile='legacy'
    motion=yaml.safe_load((share/'config/warehouse_motion.yaml').read_text())[profile]
    velocity_remaps = [('/cmd_vel','/warehouse/auto_cmd_vel')] if hardware else []
    config=str(share/'config/warehouse_nav2.yaml')
    layout=yaml.safe_load((share/'config/warehouse.yaml').read_text())
    navshare=Path(get_package_share_directory('nav2_bt_navigator'))
    nodes=[Node(package='nav2_map_server',executable='map_server',name='map_server',parameters=[config,{'yaml_filename':str(share/'config/warehouse_map.yaml'),'use_sim_time':True}],output='screen')]
    names=['map_server','planner_server','controller_server','behavior_server','bt_navigator']
    for name,pkg in [('planner_server','nav2_planner'),('controller_server','nav2_controller'),('behavior_server','nav2_behaviors'),('bt_navigator','nav2_bt_navigator')]:
        extra={'default_nav_to_pose_bt_xml':str(navshare/'behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml')} if name=='bt_navigator' else {}
        if name=='controller_server': extra.update(motion['navigation'])
        nodes.append(Node(package=pkg,executable=name,name=name,parameters=[config,extra],remappings=velocity_remaps,output='screen'))
    nodes.append(TimerAction(period=10.,actions=[Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_navigation',parameters=[{'use_sim_time':True,'autostart':True,'node_names':names,'bond_timeout':0.}],output='screen')]))
    nodes.append(Node(package='landerpi_simulation',executable='warehouse_runtime.py',parameters=[{'use_sim_time':True,'csv_path':LaunchConfiguration('csv_path'),'require_operator':hardware,**motion['runtime']}],remappings=velocity_remaps,output='screen'))
    if not hardware:
        nodes.append(Node(package='landerpi_simulation', executable='warehouse_demo.py',
            parameters=[{'quantity':int(LaunchConfiguration('quantity').perform(context))}],
            condition=IfCondition(LaunchConfiguration('autorun')), output='screen'))
    if hardware:
        nodes.append(Node(package='joy', executable='joy_node', name='warehouse_joy',
            parameters=[{'device_id': ParameterValue(LaunchConfiguration('joy_device'), value_type=int),
                         'autorepeat_rate': 20.0}], output='screen'))
        controls = {key: ParameterValue(LaunchConfiguration(key), value_type=kind)
            for key, kind in [('start_button',int),('back_button',int),('linear_axis',int),
                              ('angular_axis',int),('max_linear',float),('max_angular',float),('quantity',int)]}
        nodes.append(Node(package='landerpi_simulation', executable='warehouse_joystick.py',
                          parameters=[controls], output='screen'))
        nodes.extend([
            SetEnvironmentVariable('MACHINE_TYPE', LaunchConfiguration('machine_type')),
            Node(package='ros_robot_controller', executable='ros_robot_controller',
                 name='ros_robot_controller', parameters=[{'use_sim_time':False}], output='screen'),
            Node(package='controller', executable='odom_publisher', name='hardware_base_controller',
                 parameters=[str(Path(get_package_share_directory('controller'))/'config/calibrate_params.yaml'),
                             {'use_sim_time':False,'pub_odom_topic':False}],
                 remappings=[('controller/cmd_vel','/cmd_vel'),('cmd_vel','/hardware/unused_cmd_vel')],
                 output='screen')])
    return nodes


def generate_launch_description():
    share=Path(get_package_share_directory('landerpi_simulation'))
    layout=yaml.safe_load((share/'config/warehouse.yaml').read_text())
    return LaunchDescription([
        DeclareLaunchArgument('motion_profile',default_value='fast',description='Pure simulation motion profile: fast or legacy; hardware always legacy'),
        DeclareLaunchArgument('autorun',default_value='false',description='Automatically run pure simulation; hardware always requires gamepad START'),DeclareLaunchArgument('quantity',default_value='0',description='0 sorts all remaining cargo'),
        *[DeclareLaunchArgument(k,default_value=v) for k,v in dict(hardware='false',machine_type='MentorPi_Mecanum',joy_device='0',start_button='7',back_button='6',linear_axis='1',angular_axis='0',max_linear='0.2',max_angular='0.8').items()],
        DeclareLaunchArgument('gui',default_value='true'),DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('ros_domain_id',default_value='77'),DeclareLaunchArgument('gazebo_port',default_value='0'),
        DeclareLaunchArgument('csv_path',default_value='/tmp/landerpi_experiments/warehouse_tasks.csv'),
        SetEnvironmentVariable('ROS_DOMAIN_ID',LaunchConfiguration('ros_domain_id')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/simulation.launch.py')),launch_arguments={
            **{k:LaunchConfiguration(k) for k in ('gui','rviz','ros_domain_id','gazebo_port')},
            'onboard_camera':'true','legacy_camera':'false','world':str(share/'worlds/warehouse.world'),'spawn_x':str(layout['start'][0]),'spawn_y':str(layout['start'][1]),
            'rviz_config':str(share/'rviz/warehouse.rviz')}.items()),OpaqueFunction(function=stack)])
