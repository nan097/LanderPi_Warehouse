"""LanderPi Mecanum: real RRCLite, feedback servos, Aurora930 and joystick."""
from pathlib import Path
import os
import xml.etree.ElementTree as ET
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def setup(context):
    arg=lambda key:LaunchConfiguration(key).perform(context)
    desc=Path(get_package_share_directory('landerpi_description'))
    share=Path(get_package_share_directory('landerpi_mtc_grasp'))
    config=Path(get_package_share_directory('landerpi_moveit_config'))/'config/simulation'
    os.environ['MACHINE_TYPE']='LanderPi_Mecanum'
    model=ET.fromstring(xacro.process_file(str(desc/'urdf/landerpi.xacro')).toxml())
    # Wheel rotation is irrelevant to this stationary, base-relative arm planner.
    for joint in model.findall('joint'):
        if joint.get('name','').startswith('wheel_'):joint.set('type','fixed')
    for tag in list(model.findall('gazebo')):model.remove(tag)
    robot={'robot_description':ET.tostring(model,encoding='unicode'),'use_sim_time':False}
    load=lambda name:yaml.safe_load((config/name).read_text())
    params={**robot,'robot_description_semantic':(config/'landerpi.srdf').read_text(),
        'robot_description_kinematics':load('kinematics.yaml'),
        'robot_description_planning':load('joint_limits.yaml'),
        'planning_pipelines':['ompl'],'default_planning_pipeline':'ompl','ompl':load('ompl_planning.yaml'),
        'capabilities':'move_group/ExecuteTaskSolutionCapability',
        'publish_robot_description_semantic':True,'publish_planning_scene':True,
        'publish_geometry_updates':True,'publish_state_updates':True,'publish_transforms_updates':True,
        **load('moveit_controllers.yaml')}
    rgb=arg('rgb_topic');depth=arg('depth_topic');info=arg('rgb_info_topic')
    nodes=[
        Node(package='ros_robot_controller',executable='ros_robot_controller',name='ros_robot_controller',parameters=[{'motor_command_timeout':.6}],output='screen'),
        Node(package='controller',executable='odom_publisher',name='mtc_real_base',
            parameters=[{'machine_type':'LanderPi_Mecanum','pub_odom_topic':False,'command_timeout':.4}],
            remappings=[('controller/cmd_vel','/mtc_grasp/base_cmd_vel'),('cmd_vel','/mtc_grasp/unused_cmd_vel')],output='screen'),
        Node(package='landerpi_mtc_grasp',executable='hardware_servo.py',parameters=[{'servo_config':arg('servo_config')}],output='screen'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[robot],output='screen'),
        # map is the current robot-local grasp frame, not a navigation world map.
        Node(package='tf2_ros',executable='static_transform_publisher',arguments=['--frame-id','map','--child-frame-id','base_footprint']),
        Node(package='moveit_ros_move_group',executable='move_group',parameters=[params],output='screen'),
        Node(package='landerpi_mtc_grasp',executable='color_grasp',parameters=[params,{
            'hardware_mode':True,'target_color':arg('target_color'),
            'velocity_scale':float(arg('velocity_scale')),'acceleration_scale':float(arg('acceleration_scale')),
            'gripper_velocity_scale':.3,'gripper_acceleration_scale':.3}],output='screen'),
        Node(package='landerpi_mtc_grasp',executable='register_depth.py',
            parameters=[{'already_aligned':True}],
            remappings=[('depth',depth),('depth_info',arg('depth_info_topic')),('rgb_info',info),('aligned_depth','/mtc_grasp/aligned_depth')],output='screen'),
        Node(package='landerpi_mtc_grasp',executable='color_vision.py',parameters=[{'target_color':arg('target_color'),'use_sim_time':False}],
            remappings=[('/warehouse_camera/camera/image_raw',rgb),('/warehouse_camera/camera/depth/image_raw','/mtc_grasp/aligned_depth'),('/warehouse_camera/camera/camera_info',info)],output='screen'),
        Node(package='joy',executable='joy_node',parameters=[{'device_id':int(arg('joy_device')),'autorepeat_rate':20.}],output='screen'),
        Node(package='landerpi_mtc_grasp',executable='mtc_joystick.py',parameters=[{k:int(arg(k)) for k in ('start_button','back_button','linear_axis','angular_axis','lateral_axis')}],output='screen')]
    if arg('start_vision').lower()!='true':nodes=[node for index,node in enumerate(nodes) if index not in (7,8)]
    if arg('start_drivers').lower()!='true':nodes=nodes[2:]
    if arg('start_joystick').lower()!='true':nodes=nodes[:-2]
    if arg('start_camera').lower()=='true':
        nodes.append(Node(package='deptrum-ros-driver-aurora930',executable='aurora930_node',namespace='aurora',
            parameters=[{'rgb_enable':True,'depth_enable':True,'ir_enable':True,'rgbd_enable':True,'align_mode':True,
                'point_cloud_enable':False,'rgb_fps':15,'ir_fps':15,'depth_correction':True,
                'minimum_filter_depth_value':150,'maximum_filter_depth_value':4000,'resolution_mode_index':2}],
            remappings=[('/aurora/rgb/image_raw',rgb),('/aurora/depth/image_raw',depth),('/aurora/rgb/camera_info',info),('/aurora/ir/camera_info',arg('depth_info_topic'))],output='screen'))
        for frame in ('depth_camera_link','depth_camera_link_1','rgb_camera_link_1'):
            nodes.append(Node(package='tf2_ros',executable='static_transform_publisher',
                arguments=['--x','0','--y','0','--z','0','--roll','-1.57','--pitch','0','--yaw','-1.57',
                    '--frame-id','depth_cam_link','--child-frame-id',frame]))
    return nodes

def generate_launch_description():
    servo=Path(get_package_share_directory('servo_controller'))/'config/servo_controller.yaml'
    defaults=dict(target_color='red',ros_domain_id='80',velocity_scale='0.2',acceleration_scale='0.2',
        servo_config=str(servo),start_drivers='true',start_joystick='true',start_camera='true',start_vision='true',joy_device='0',start_button='9',back_button='8',
        linear_axis='1',lateral_axis='0',angular_axis='2',
        rgb_topic='/ascamera/camera_publisher/rgb0/image',depth_topic='/ascamera/camera_publisher/depth0/image_raw',
        rgb_info_topic='/ascamera/camera_publisher/rgb0/camera_info',depth_info_topic='/ascamera/camera_publisher/depth0/camera_info')
    return LaunchDescription([*[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()],
        SetEnvironmentVariable('MACHINE_TYPE','LanderPi_Mecanum'),
        SetEnvironmentVariable('ROS_DOMAIN_ID',LaunchConfiguration('ros_domain_id')),OpaqueFunction(function=setup)])
