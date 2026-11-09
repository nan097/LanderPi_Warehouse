"""Gazebo Classic + ros2_control + MoveIt, with no hardware nodes."""
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
import struct
import socket
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def setup(context):
    requested_port = int(LaunchConfiguration('gazebo_port').perform(context))
    if requested_port != 0 and not 1024 <= requested_port <= 65535:
        raise ValueError('gazebo_port must be 0 (automatic) or in [1024, 65535]')
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        try:
            probe.bind(('127.0.0.1', requested_port))
        except OSError as exc:
            raise RuntimeError(f'Gazebo port {requested_port} unavailable; use gazebo_port:=0') from exc
        port = probe.getsockname()[1]
    master_uri = f'http://127.0.0.1:{port}'
    share = Path(get_package_share_directory('landerpi_simulation'))
    moveit = Path(get_package_share_directory('landerpi_moveit_config')) / 'config/simulation'
    description = xacro.process_file(str(share / 'urdf/landerpi_sim.urdf.xacro'), mappings={'onboard_camera':LaunchConfiguration('onboard_camera').perform(context)}).toxml()
    # The original URDF uses locked wheel limits; planar plugin controls the body.
    # Lock wheels for this ideal holonomic simulation, rather than invalid zero-effort joints.
    model = ET.fromstring(description)
    description_share = Path(get_package_share_directory('landerpi_description'))
    for mesh in model.iter('mesh'):
        uri = mesh.get('filename', '')
        if uri.startswith('package://landerpi_description/'):
            asset = description_share / uri.removeprefix('package://landerpi_description/')
            if not asset.is_file():
                raise FileNotFoundError(asset)
            mesh.set('filename', asset.as_uri())
    for joint in model.findall('joint'):
        if joint.get('name', '').startswith('wheel_'):
            joint.set('type', 'fixed')
    # Gazebo uses conservative mesh bounds as boxes for stable ODE contacts.
    # Keep original STL visuals and all joint transforms.
    # Preserve the gripper's concave opening; a bounding box fills its jaw gap
    # and incorrectly prevents entry into small containers.
    gripper_collisions = {id(c) for link in model.findall('link')
                          if link.get('name') == 'gripper_link'
                          for c in link.findall('collision')}
    for collision in model.iter('collision'):
        if id(collision) in gripper_collisions:
            continue
        mesh = collision.find('geometry/mesh')
        if mesh is None:
            continue
        path = Path(mesh.get('filename').removeprefix('file://'))
        raw = path.read_bytes()
        count = struct.unpack_from('<I', raw, 80)[0]
        if len(raw) != 84 + count * 50:
            raise ValueError(f'Expected binary STL: {path}')
        vertices = [struct.unpack_from('<fff', raw, 84 + face * 50 + 12 + point * 12)
                    for face in range(count) for point in range(3)]
        lower = [min(point[axis] for point in vertices) for axis in range(3)]
        upper = [max(point[axis] for point in vertices) for axis in range(3)]
        center = [(lo + hi) / 2 for lo, hi in zip(lower, upper)]
        origin = collision.find('origin')
        if origin is None:
            origin = ET.SubElement(collision, 'origin', xyz='0 0 0', rpy='0 0 0')
        if any(abs(float(v)) > 1e-9 for v in origin.get('rpy', '0 0 0').split()):
            raise ValueError('Rotated collision origins require explicit box transform')
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
        origin.set('xyz', ' '.join(str(v + c) for v, c in zip(xyz, center)))
        geometry = collision.find('geometry')
        geometry.remove(mesh)
        ET.SubElement(geometry, 'box', size=' '.join(str(max(hi-lo, 0.001)) for lo,hi in zip(lower,upper)))
    # The ideal planar base has no wheel friction model. Remove lateral drag.
    for link in model.findall('link'):
        gazebo = ET.SubElement(model, 'gazebo', reference=link.get('name'))
        friction = '0.0'
        ET.SubElement(gazebo, 'mu1').text = friction
        ET.SubElement(gazebo, 'mu2').text = friction
    description = ET.tostring(model, encoding='unicode')
    robot = {'robot_description': description, 'use_sim_time': True}
    def load(name):
        return yaml.safe_load((moveit / name).read_text())
    moveit_params = {
        **robot,
        'robot_description_semantic': (moveit / 'landerpi.srdf').read_text(),
        'robot_description_kinematics': load('kinematics.yaml'),
        'robot_description_planning': load('joint_limits.yaml'),
        'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl',
        'ompl': load('ompl_planning.yaml'),
        'publish_robot_description': False, 'publish_robot_description_semantic': True,
        'publish_planning_scene': True, 'publish_geometry_updates': True,
        'publish_state_updates': True, 'publish_transforms_updates': True,
        **load('moveit_controllers.yaml'),
    }
    mtc_enabled = LaunchConfiguration('mtc_grasp').perform(context).lower() == 'true'
    if mtc_enabled:
        moveit_params['capabilities'] = 'move_group/ExecuteTaskSolutionCapability'
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                 arguments=['-entity', 'landerpi', '-topic', 'robot_description', '-z', '0.02', '-x', LaunchConfiguration('spawn_x'), '-y', LaunchConfiguration('spawn_y')], output='screen')
    def spawner(controller):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[controller, '--controller-manager-timeout', '60'],
                    parameters=[{'use_sim_time': True}], output='screen')
    jsb = spawner('joint_state_broadcaster')
    arm = spawner('arm_controller')
    gripper = spawner('gripper_controller')
    # Start each controller only after its predecessor succeeded.
    def after_success(event, _):
        if event.returncode != 0:
            from launch.actions import EmitEvent
            from launch.events import Shutdown
            return [EmitEvent(event=Shutdown(reason='Spawn/controller activation failed'))]
        return {spawn: [jsb], jsb: [arm], arm: [gripper], gripper: []}[event.action]
    return [
        SetEnvironmentVariable('GAZEBO_MASTER_URI', master_uri),
        # Gazebo 11 requests /scene_info via Ignition transport. Its discovery
        # is independent of GAZEBO_MASTER_URI: isolate both server and client.
        SetEnvironmentVariable('IGN_PARTITION', f'landerpi_{port}'),
        SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', str(share) + ':/usr/share/gazebo-11:' + os.environ.get('GAZEBO_RESOURCE_PATH', '')),
        LogInfo(msg=f'[SIM] Gazebo master: {master_uri}; IGN_PARTITION=landerpi_{port}; ROS_DOMAIN_ID=' + LaunchConfiguration('ros_domain_id').perform(context)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('gazebo_ros')) / 'launch/gazebo.launch.py')),
            launch_arguments={'world': LaunchConfiguration('world'), 'gui': LaunchConfiguration('gui'), 'verbose': 'true', 'server_required': 'true', 'gui_required': LaunchConfiguration('gui')}.items()),
        Node(package='tf2_ros', executable='static_transform_publisher', arguments=['--x','0','--y','0','--z','0','--yaw','0','--pitch','0','--roll','0','--frame-id','map','--child-frame-id','odom']),
        Node(package='tf2_ros', executable='static_transform_publisher', condition=IfCondition(LaunchConfiguration('legacy_camera')), arguments=['--x','0.75','--y','0','--z','1','--yaw','0','--pitch','1.57079632679','--roll','0','--frame-id','map','--child-frame-id','camera_link']),
        Node(package='tf2_ros', executable='static_transform_publisher', condition=IfCondition(LaunchConfiguration('legacy_camera')), arguments=['--yaw','-1.57079632679','--pitch','0','--roll','-1.57079632679','--frame-id','camera_link','--child-frame-id','camera_optical_frame']),
        Node(package='landerpi_simulation', executable='scan_filter.py', parameters=[{'use_sim_time':True}], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[robot], output='screen'),
        Node(package='moveit_ros_move_group', executable='move_group', parameters=[moveit_params], output='screen', condition=IfCondition(LaunchConfiguration('moveit'))),
        *([Node(package='landerpi_mtc_grasp', executable='color_grasp',
                prefix=os.environ.get('LANDERPI_MTC_DEBUG_PREFIX',''),
                parameters=[moveit_params, {'target_color':LaunchConfiguration('target_color'),
                    **{key:ParameterValue(LaunchConfiguration(key), value_type=kind) for key,kind in
                       [('velocity_scale',float),('acceleration_scale',float),('gripper_velocity_scale',float),
                        ('gripper_acceleration_scale',float),('place_x',float),('place_y',float),('place_z',float),('observe_joint2',float),('observe_joint3',float),('observe_joint4',float)]}}],
                output='screen')] if mtc_enabled else []),
        Node(package='rviz2', executable='rviz2', arguments=['-d', LaunchConfiguration('rviz_config')], parameters=[moveit_params], condition=IfCondition(LaunchConfiguration('rviz'))),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after_success)),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=after_success)),
        RegisterEventHandler(OnProcessExit(target_action=arm, on_exit=after_success)),
        RegisterEventHandler(OnProcessExit(target_action=gripper, on_exit=after_success)),
        Node(package='landerpi_simulation', executable='motion_demo.py', parameters=[{'use_sim_time': True}], output='screen', condition=IfCondition(LaunchConfiguration('demo'))),
        spawn,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('world', default_value=str(Path(get_package_share_directory('landerpi_simulation')) / 'worlds/sorting.world')),
        DeclareLaunchArgument('onboard_camera', default_value='false'),
        DeclareLaunchArgument('legacy_camera', default_value='true'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=str(Path(get_package_share_directory('landerpi_simulation')) / 'rviz/simulation.rviz')),
        *[DeclareLaunchArgument(key,default_value=value) for key,value in dict(velocity_scale='0.7',acceleration_scale='0.65',gripper_velocity_scale='0.8',gripper_acceleration_scale='0.8',place_x='0.08',place_y='0.24',place_z='0.07',observe_joint2='0.1',observe_joint3='-0.8',observe_joint4='-2.0').items()],
        DeclareLaunchArgument('mtc_grasp', default_value='false'),
        DeclareLaunchArgument('target_color', default_value='red'),
        DeclareLaunchArgument('moveit', default_value='true'),
        DeclareLaunchArgument('demo', default_value='false', description='Repeat simulated arm/gripper/base preview'),
        DeclareLaunchArgument('gazebo_port', default_value='0', description='0 chooses an available local TCP port'),
        DeclareLaunchArgument('ros_domain_id', default_value='42', description='Isolated simulation ROS domain'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable('MACHINE_TYPE', 'LanderPi_Mecanum'),
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        OpaqueFunction(function=setup),
    ])
