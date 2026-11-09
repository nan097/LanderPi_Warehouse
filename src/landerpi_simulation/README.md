# LanderPi simulation

完整任务入口、接口、仿真假设和验收命令见 [完整仿真说明](../../docs/full_simulation.md)。

# landerpi_simulation

Gazebo Classic 11 / ROS 2 Humble 仿真包，复用 landerpi_description 五轴模型和夹爪，不启动串口或真实驱动。

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select landerpi_description landerpi_moveit_config landerpi_simulation landerpi_bringup
source install/setup.bash
ros2 launch landerpi_bringup simulation.launch.py
```

无界面：

```bash
ros2 launch landerpi_bringup simulation.launch.py gui:=false rviz:=false
```

接口：

| 接口 | 类型 / 作用 |
|---|---|
| /cmd_vel | geometry_msgs/msg/Twist，全向平面运动输入 |
| /odom | nav_msgs/msg/Odometry，模拟里程计 |
| /scan | sensor_msgs/msg/LaserScan，360 点 CPU 雷达 |
| /joint_states | sensor_msgs/msg/JointState，Gazebo 关节反馈 |
| /clock | rosgraph_msgs/msg/Clock，仿真时钟 |
| /arm_controller/follow_joint_trajectory | control_msgs/action/FollowJointTrajectory，五轴机械臂 |
| /gripper_controller/follow_joint_trajectory | control_msgs/action/FollowJointTrajectory，r_joint |
| /plan_kinematic_path | moveit_msgs/srv/GetMotionPlan，MoveIt 规划 |

TF：scan_filter 将 planar_move 的 /odom 同源转换为唯一 odom → base_footprint TF；robot_state_publisher 发布模型 TF。启动参数 gui/rviz/moveit 均默认 true；demo 默认 false；gazebo_port 默认 0（自动选择空闲端口）；ros_domain_id 默认 42。所有节点使用仿真时钟。

世界包含地面、两张工位台和一个箱体。坐标、初始姿态及限制是仿真参数，不是硬件标定结果。底盘使用 Gazebo 自带 planar_move 理想全向速度模型，轮子在仿真中固定，不模拟麦轮接触力；接触摩擦设为零。STL 外观保留，碰撞体使用 STL 包围盒。

完整任务入口 sort_simulation.launch.py 已接入 RGB-D、Nav2、MoveIt、物体抓放约束和 ExecuteSort 调度。运行 `bash tools/run_sort_demo.sh` 可启动可见的自动搬运任务；`task_system.launch.py` 保留独立 dry_run 用途。完整入口使用 ROS_DOMAIN_ID=75，下面基础演示使用 42。

测试：`bash tools/run_simulation_test.sh`（工作区根目录），无界面启动、控制并自动退出；日志在 /tmp/landerpi_simulation_smoke_<pid>.log。

Dependencies：见 package.xml。Failure Behavior：模型网格缺失时启动报错，实体生成或控制器激活失败时关闭 launch。Gazebo 和 MoveIt 的系统依赖已安装于当前主机。

完整运行验证见 [仿真验证记录](../../docs/simulation_test_report.md)。机械臂、夹爪、横移和 MoveIt 规划已通过回归。

## 可见动作演示

```bash
bash tools/run_simulation.sh demo:=true
```

在工作区根目录执行，打开 Gazebo/RViz，循环演示机械臂旋转、夹爪开闭、底盘往返横移。演示使用模拟 Action 的完成结果推进；它不是完整分拣任务。

手动使用 MoveIt 时关闭演示（默认 demo=false）。启动日志打印本次 Gazebo master 地址，既有 Gazebo 进程不会被自动终止。显式指定已占用的 gazebo_port 时直接报告错误；gzserver 或控制器启动失败时结束整套 launch。

从另一个终端查询这套仿真时：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 topic list
ros2 action list -t
```

若使用 ros_domain_id 参数更换域，查询终端必须设置为相同值。Gazebo 客户端由 launch 自动使用本次选定端口。
