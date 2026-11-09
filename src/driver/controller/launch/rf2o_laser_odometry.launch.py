from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService

#ROS 2 激光里程计节点
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic' : '/scan_raw',#激光雷达数据主题
                'odom_topic' : '/odom_rf2o',#里程计数据主题
                'publish_tf' : False,#是否发布TF变换
                'base_frame_id' : 'base_footprint',#机器人底盘坐标系
                'odom_frame_id' : 'odom',#里程计坐标系
                'init_pose_from_topic' : '',#初始化位姿主题
                'freq' : 10.0}],
            arguments=['--ros-args', '--log-level', 'WARN'],
        ),
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
