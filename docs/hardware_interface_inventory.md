# 硬件接口盘点（Phase 0，源码证据）

环境：主机已安装 ROS 2 Humble。以下是源码确认的候选接口，尚未启动实物验证；命名空间和 remap 可能改变最终全名。不得将源码存在等同于硬件在线。

| Node | Topic / Service / Action | Type | 发布者 / 订阅者 | 来源 |
|---|---|---|---|---|
| odom_publisher | controller/cmd_vel、cmd_vel | geometry_msgs/msg/Twist | 订阅 | src/driver/controller/controller/odom_publisher_node.py |
| odom_publisher | odom_raw | nav_msgs/msg/Odometry | 发布 | 同上；pub_odom_topic 参数控制 |
| odom_publisher | ros_robot_controller/set_motor | ros_robot_controller_msgs/msg/MotorsState | 发布；STM32 节点订阅 | 同上 |
| ros_robot_controller | ~/imu_raw | sensor_msgs/msg/Imu | 发布 | src/driver/ros_robot_controller/ros_robot_controller/ros_robot_controller_node.py |
| ros_robot_controller | ~/battery | std_msgs/msg/UInt16 | 发布 | 同上 |
| ros_robot_controller | ~/bus_servo/set_position | ros_robot_controller_msgs/msg/ServosPosition | 订阅 | 同上 |
| ros_robot_controller | ~/bus_servo/get_state | ros_robot_controller_msgs/srv/GetBusServoState | 服务端 | 同上 |
| controller_manager | ~/joint_states | sensor_msgs/msg/JointState | 发布 | src/driver/servo_controller/servo_controller/controller_manager.py |
| controller_manager | ~/servo_states | servo_controller_msgs/msg/ServoStateList | 发布 | 同上 |
| controller_manager | servo_controller | servo_controller_msgs/msg/ServosPosition | 订阅 | 同上 |
| controller_manager | joint_controller | sensor_msgs/msg/JointState | 订阅 | 同上 |
| 旧 trajectory controller | arm_controller/follow_joint_trajectory | control_msgs/action/FollowJointTrajectory | Action 服务端 | src/driver/servo_controller/servo_controller/joint_trajectory_action_controller.py；配置注册见 controller_manager.py |

## 已发现的关键差异

- 当前 arm.urdf.xacro 为 joint1…joint5 五个转动关节，末端为 end_effector_link；不存在文档示例的 arm_base_link/tool0。六轴实物与五轴模型的对应关系：TODO_CONFIRM。
- 当前夹爪配置为 r_joint / w_joint；URDF 含联动指节，标准 GripperCommand 和位置单位：TODO_CONFIRM。
- JointState 来自 ServoManager.get_position() 返回的缓存；set_position() 更新的是已发送命令值。当前不能视为实测位置反馈，标记 OPEN_LOOP_ASSUMPTION。
- 旧 FJT 存在，但取消和执行结果语义需要审查，不能以它存在就认为满足新架构。
- controller 默认 base_frame_id=base_footprint、odom_frame_id=odom；原里程计直接 TF 发送逻辑被注释。EKF 配置和启动模式需运行时确认唯一 TF 发布者。
- 保留了活动 src/driver/servo_controller 版本；archive/src copy 内对应包与它不同，不能直接覆盖。
- core_hardware.launch.py 目前只启动 STM32 节点，不包含完整底盘、舵机及外设链。
- 原驱动存在 /home/ubuntu 路径、环境变量和预编译 .so 依赖，未改动底层实现。

## 待确认

雷达型号、scan Topic/Frame；RGB、Depth、CameraInfo、optical frame、RGB/Depth 对齐；硬件反馈可用性；舵机方向/零位/限位；工位坐标；相机外参；运行时全部接口与 TF 发布者。

实物启动后保存以下输出，并补充 Publisher / Subscriber 的最终全名：

```bash
ros2 node list
ros2 topic list -t
ros2 service list -t
ros2 action list -t
ros2 param list
ros2 topic info /controller_manager/joint_states -v
ros2 topic echo /controller_manager/joint_states --once
ros2 action info /arm_controller/follow_joint_trajectory
ros2 run tf2_tools view_frames
```
