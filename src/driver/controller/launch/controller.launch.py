import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.conditions import IfCondition
from nav2_common.launch import ReplaceString
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction, OpaqueFunction

def launch_setup(context):
    #参数声明
    compiled = os.environ['need_compile']# 获取环境变量，判断是否为编译版本
    namespace = LaunchConfiguration('namespace', default='')# 获取命名空间参数
    use_namespace = LaunchConfiguration('use_namespace', default='false').perform(context)# 获取是否使用命名空间参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')# 获取是否使用仿真时间参数
    enable_odom = LaunchConfiguration('enable_odom', default='true')# 获取是否启用里程计参数
    map_frame = LaunchConfiguration('map_frame', default='map')# 获取地图坐标系参数
    odom_frame = LaunchConfiguration('odom_frame', default='odom')# 获取里程计坐标系参数
    base_frame = LaunchConfiguration('base_frame', default='base_footprint')# 获取机器人基座坐标系参数
    imu_frame = LaunchConfiguration('imu_frame', default='imu_link')# 获取IMU坐标系参数
    frame_prefix = LaunchConfiguration('frame_prefix', default='')# 获取坐标系前缀参数

    #参数声明动作
    namespace_arg = DeclareLaunchArgument('namespace', default_value=namespace)# 获取命名空间参数
    use_namespace_arg = DeclareLaunchArgument('use_namespace', default_value=use_namespace)# 获取是否使用命名空间参数
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time)# 获取是否使用仿真时间参数
    enable_odom_arg = DeclareLaunchArgument('enable_odom', default_value=enable_odom)# 获取是否启用里程计参数
    map_frame_arg = DeclareLaunchArgument('map_frame', default_value=map_frame)# 获取地图坐标系参数
    odom_frame_arg = DeclareLaunchArgument('odom_frame', default_value=odom_frame)
    base_frame_arg = DeclareLaunchArgument('base_frame', default_value=base_frame)
    imu_frame_arg = DeclareLaunchArgument('imu_frame', default_value=imu_frame)
    frame_prefix_arg = DeclareLaunchArgument('frame_prefix', default_value=frame_prefix)

    # 根据是否为编译版本设置包路径
    if compiled == 'True':
        peripherals_package_path = get_package_share_directory('peripherals')
        controller_package_path = get_package_share_directory('controller')
        servo_controller_package_path = get_package_share_directory('servo_controller')
    else:
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'
        controller_package_path = '/home/ubuntu/ros2_ws/src/driver/controller'
        servo_controller_package_path = '/home/ubuntu/ros2_ws/src/driver/servo_controller'

    # 包含里程计发布器启动文件
    odom_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(controller_package_path, 'launch/odom_publisher.launch.py')
        ]),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'imu_frame': imu_frame,
            'frame_prefix': frame_prefix,
            'base_frame': base_frame,
            'odom_frame': odom_frame
        }.items()
    )

    # 包含IMU滤波器启动文件
    imu_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(peripherals_package_path, 'launch/imu_filter.launch.py')
        ])
    )
    # EKF滤波器节点
    if use_namespace == 'false':
        ekf_param = ReplaceString(source_file=os.path.join(controller_package_path, 'config/ekf.yaml'), replacements={'namespace/': ''})
    else:
        ekf_param = ReplaceString(source_file=os.path.join(controller_package_path, 'config/ekf.yaml'), replacements={"namespace/": (namespace, '/')})
    
    # EKF滤波器节点
    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_param, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('odometry/filtered', 'odom'),# 重映射到odom话题
            ('cmd_vel', 'controller/cmd_vel')# 重映射速度命令
        ],
        condition=IfCondition(enable_odom),# 条件启动
    )

    #舵机控制器
    servo_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(servo_controller_package_path, 'launch/servo_controller.launch.py')
        ]),
        launch_arguments={
            'base_frame': base_frame,
        }.items()
    )

    return [
        namespace_arg,
        use_namespace_arg,
        use_sim_time_arg,
        enable_odom_arg,
        odom_frame_arg,
        base_frame_arg,
        map_frame_arg,
        imu_frame_arg,
        frame_prefix_arg,
        imu_filter_launch,
        odom_publisher_launch,
        ekf_filter_node,
        servo_controller_launch
    ]

#生成启动描述
def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
