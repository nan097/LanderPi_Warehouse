# TF 契约（待运行时验证）

| 变换 | 源码/设计依据 | 计划唯一发布者 | 状态 |
|---|---|---|---|
| map → odom | 上层导航契约 | localization 或 SLAM | 尚未接入 |
| odom → base_footprint | controller 默认参数 | EKF 或原 odometry，选择其一 | TODO_CONFIRM_RUNTIME |
| base_footprint → base_link → 底盘、传感器 | src/landerpi_description/urdf/mecanum.xacro | robot_state_publisher | 源码存在；实机外参 TODO_CALIBRATE |
| back_shell_black_link → link1 → … → link5 | arm.urdf.xacro，joint1…joint5 | robot_state_publisher + JointState | 五轴模型；反馈是命令缓存 |
| link5 → end_effector_link / gripper_link | arm.urdf.xacro 固定关节 | robot_state_publisher | 源码存在 |
| 夹爪联动链 | gripper.urdf.xacro | robot_state_publisher | 联动关节与反馈映射 TODO_CONFIRM |

不得新增虚构的 arm_base_link/tool0 来掩盖模型差异。每条动态 TF 只能有一个发布者。相机 optical frame、传感器标定及完整运行时 TF 树尚未确认。

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link end_effector_link
```
