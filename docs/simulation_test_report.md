# 仿真验收记录

日期：2026-10-02。ROS 2 Humble、Gazebo Classic 11、MoveIt 2.5.10。完整任务基线已通过，验收进程退出码为 0。

## 实际完成的任务

导航到取货工位 → 里程计精对准 → RGB-D 识别定位 → MoveIt 碰撞规划与抓取 → 携物导航 → 物理放置 → 返回起点。正常任务不调用 SetEntityState；测试仅用它布置每轮箱体和注入缺失目标。

| 检查 | 实际结果 |
|---|---|
| 相关软件包构建 | 7 个包构建成功；新 RViz 配置再次构建成功 |
| 框架静态检查 | 5/5 通过 |
| 取消接收握手与执行期间取消回归 | 2/2 通过，子目标停止且取得终态 |
| 随机 RGB-D 抓取 | 20/20 成功：10 次独立抓放 + 10 次完整搬运 |
| 完整任务 | 10/10 成功，每次数量 1，包含返航 |
| RGB-D 三维定位误差 | 20 次测量的最大误差 7.35 mm |
| 抓取前工具与物体中心距离 | 最大 7.90 mm，满足附着要求 ≤30 mm |
| 完整任务耗时 | 平均 115.90 s，范围 110.69–122.57 s；为当前主机墙钟时间 |
| 仅规划模式 | 未执行机械臂运动 |
| 非法数量、任务互斥 | 拒绝目标 |
| 取消、缺失目标、故障复位 | 通过；错误码分别为 502、301；检测有限重试 |
| 实际落点、返航、归位、场景清理 | 每轮均核对 Gazebo 箱体、附着状态、odom、关节反馈、MoveIt 场景 |

随机种子 42；初始中心 x∈[0.664,0.676] m、y∈[-0.006,0.006] m，z=0.185 m。此结果仅代表该限定区域和理想仿真模型，不外推更大抓取区域或真实硬件成功率。

CSV 共 12 条记录：10 条成功任务，另 2 条为主动取消和缺失目标故障注入，按预期失败。不会将故障注入混入完整任务成功率。

## 证据

- [任务 CSV](experiments/simulation_baseline_10_tasks.csv)
- [测量与耗时汇总](experiments/simulation_baseline_summary.json)
- [验收终端输出](experiments/simulation_baseline_stdout.txt)（从第 3 次独立抓放起保存，包含最终通过声明）
- 完整运行日志（原始日志未随源码发布）
- 原始日志 `/tmp/landerpi_sort_test_49877.log`，原始 CSV `/tmp/landerpi_sort_acceptance_49877.csv`。
- [时序修复组合回归](experiments/simulation_timing_fix_regression.csv)：3 次抓放、2 次完整任务及故障恢复整体通过。

可重新计算：

```bash
python3 tools/summarize_sort_acceptance.py \
  docs/experiments/simulation_baseline.log.gz \
  docs/experiments/simulation_baseline_10_tasks.csv
```

## 关键修复

- 自动选择空闲 Gazebo master 端口，隔离 ROS 域，防止图形客户端连接旧服务器。
- 五轴 URDF、ros2_control、夹爪、MoveIt 控制器与 Planning Scene 完整接通。
- 修正红箱材质、RGB/深度同步和原始时间戳，实际视觉定位代替固定目标坐标。
- 正确销毁抓取与驻车 fixed joint；机械臂动作时驻车，导航前释放。
- 放置在目标上方 4 cm 解除约束，由重力落桌并核对实际落点。
- 修复长时间运行后 Nav2 内部 TF 缓存停滞：雷达保留测量时间，等测量后 50 ms 的 TF 就绪，并延迟到测量后 200 ms 交给 Nav2。单独切换 DDS 未能消除该问题；此时序修复后的组合及完整基线均通过。
- odom 与动态 TF 同源且唯一；控制器拒绝过期 TF，在代价地图锁内检查扫掠碰撞；工作区使用 Cyclone DDS 本机配置。
- 取消时等子目标接收和终态，避免接收握手期间遗漏运动目标。

## 复现与范围

```bash
bash tools/build_simulation.sh
bash tools/run_sort_demo.sh
# 无界面完整基线并自动退出
bash tools/run_sort_test.sh --runs 10 --grasp-runs 10 --fault-check
```

完整入口、9 个 Action、仿真假设见 [完整仿真说明](full_simulation.md)。地图/odom 为仿真真值，使用固定俯视 RGB-D 相机、五轴位置 IK 和理想刚性抓取；未接入实物驱动、SLAM/AMCL 或任意 6D 抓取。其他上层包的硬件后端仍保留框架，不宣称完成硬件迁移。

完整任务图形演示也已成功完成并返航：[可见演示 CSV](experiments/simulation_visual_demo.csv)、可见演示日志（原始日志未随源码发布）。截图确认 Gazebo 机器人与两工位、RViz 地图/场景、路径、雷达及 RGB-D 画面可见，Global Status 和 MotionPlanning Status 均为 Ok。当前预览由独立会话保留，ROS 域 76，启动器 PID 53038，Gazebo master 端口 47353。

图形证据：[Gazebo 抓取](screenshots/gazebo_full_task_pick.png)、[RViz 抓取与相机](screenshots/rviz_full_task_pick.png)、[Gazebo 完成](screenshots/gazebo_full_task_complete.png)、[RViz 完成](screenshots/rviz_full_task_complete.png)。

当前系统 MoveIt 在退出时仍可能出现 class_loader 析构错误或段错误，属于已有系统库问题；运行中的规划、执行和物理验收另行核对，不宣称该退出问题已修复。

运行日志中的 Octomap 传感器缺失及 /recognize_objects 未提供对应未启用的功能；本任务用显式 Planning Scene 和自己的 RGB-D Action。虚拟机 RViz 地图着色器有一次 GLSL 校验告警，截图仍能看到场景/路径/雷达，相机与任务运行正常；未修改系统图形库。
