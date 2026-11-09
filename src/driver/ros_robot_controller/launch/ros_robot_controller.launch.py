from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    imu_frame = LaunchConfiguration('imu_frame', default='imu_link')#IMU坐标系
    imu_frame_arg = DeclareLaunchArgument('imu_frame', default_value=imu_frame)#IMU坐标系参数声明

 
    ros_robot_controller_node = Node(
        package='ros_robot_controller',
        executable='ros_robot_controller',#可执行文件名字
        output='screen',
        parameters=[{'imu_frame': imu_frame}]#IMU坐标系参数传递
    )

    return LaunchDescription([
        imu_frame_arg,
        ros_robot_controller_node
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
