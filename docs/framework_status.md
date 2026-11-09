# 基础框架状态

已完成基础框架和仿真任务适配：完整入口连接真实的 Gazebo、Nav2 和 MoveIt，执行感知、抓取、运输、放置与返航。设计文档是架构参考，本文记录实际实现范围。硬件后端仍待接入。完整仿真基线已通过：20/20 次随机抓取、10/10 次完整任务；详见 [仿真验收记录](simulation_test_report.md)。

| Package | 当前实现 | 运行接口 / 配置 / TF |
|---|---|---|
| landerpi_task_interfaces | 9 Action、RobotStatus、ResetFaults，按文档逐字提取 | 仅类型定义 |
| landerpi_robot_adapter | 4 个 C++ 可启动骨架 | 不订阅、不发布运动，不开放 FJT/Gripper；hardware_interfaces.yaml 是盘点数据 |
| landerpi_moveit_config | simulation/ 下已接入五轴 SRDF、KDL、OMPL、模拟控制器 | 由 simulation.launch.py 加载；实机配置仍待确认 |
| landerpi_navigation | station_navigator_node 骨架、stations.yaml | 尚未连接 Nav2，无 Action 服务端 |
| landerpi_fine_alignment | fine_alignment_server 骨架 | 尚无视觉闭环或底盘命令 |
| landerpi_perception | 节点骨架、Detector 抽象、HSV 检测器、有效深度投影 | RGB-D 订阅/TF/DetectObject 尚未连接；AprilTag/QR/YOLO 明确未实现 |
| landerpi_manipulation | manipulation_server、planning_scene_bridge 骨架及 Backend 契约 | 抓放、MoveIt/Legacy、Planning Scene 尚未连接 |
| landerpi_robot_monitor | 周期发布全 false 的 RobotStatus | /robot/status，landerpi_task_interfaces/msg/RobotStatus，深度 10；ResetFaults 尚未实现 |
| landerpi_task_manager | dry_run FSM、数量循环、返航分支、单任务互斥、取消、总超时 | /task/execute_sort；execution_mode、dry_run_step_sec、total_timeout_sec、csv_path；无 TF/运动指令 |
| landerpi_experiment_tools | CSV 写入组件 | 被 task_manager 调用，CSV 明确记录 execution_mode；benchmark_runner 骨架 |
| landerpi_bringup | task_system / upper_stack / simulation launch | task_system 为 dry_run；simulation 为基础预览；sort_simulation 为完整任务 |
| landerpi_simulation | 全向底盘、机械臂、夹爪、雷达、RGB-D、Nav2、MoveIt 与抓放 | sort_runtime 实现全部 9 个 Action、状态监控、故障复位和任务 CSV；见完整仿真说明 |

## 默认启动语义

任务管理器在 dry_run 中跳过硬件就绪验证并模拟每个阶段；机器人监视器仍持续 robot_ready=false。成功结果只表示状态序列完成，不代表导航、感知或抓取成功。计时器只驱动模拟步骤，不能用于判定实机完成。

dry_run CSV 保留最小任务字段。完整仿真 CSV 另记录阶段耗时、软件版本、结果和错误码；仿真适配器实现子 Action 取消传播、重试、故障锁定与复位。上述 navigation/perception/manipulation 等独立包的硬件后端仍是骨架，仿真运行接口集中在 landerpi_simulation/sort_runtime.py。

## 目录整理及回滚

- src/driver 中保留已有版本，从原 src copy 补入缺失 controller、ros_robot_controller、kinematics、kinematics_msgs、sdk。
- peripherals、calibration 从原 src copy 复制到 src。
- src/landerpi_description 保留现有路径，避免仅为目录形式改动引用。
- 原 src copy 和 ros2_arm-master 移至 archive/；原 6 个演示/分析包移至 archive/legacy_packages/。
- 根目录临时 URDF、日志、图片移至 archive/。archive/COLCON_IGNORE 排除重复包及旧演示。
- 原 README 保留为 docs/hardware_migration_original.md。
- 本轮未修改现有驱动内部实现。归档内容可按原路径移回；复制补齐的包仍有原始备份。

目录保留了底层 ROS package 的历史元数据；不在本轮重写旧包依赖。

用户后续明确当前不接硬件，先跑仿真。仿真入口独立于 dry_run，现有五轴模型用于软件验证。六轴实物问题不阻塞此仿真阶段。
