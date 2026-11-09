# 完整分拣仿真

本入口实现设计文档的单机 MVP：`导航 → 精定位 → RGB-D 检测 → MoveIt 抓取 → 运输 → 放置 → 返航`。仅使用仿真设备，不启动 STM32、串口、实物舵机或真实相机驱动。

## 运行

当前机器已在 `.deps/nav2_root` 安装 Nav2 与 Cyclone DDS 依赖文件，无需 sudo。首次换机器可执行 `python3 tools/install_local_nav2.py`，前提是 ROS 2 Humble 已安装且 apt 的 ROS 软件源可用。此脚本下载 Debian 包到项目目录，不安装系统软件。

也可以在工作区中执行 `source /opt/ros/humble/setup.bash` 后直接运行 `colcon build`，构建会自动查找上述本地依赖。即使终端设置了 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`，也无需先手动添加依赖前缀。运行仿真仍使用下方脚本加载动态库和 DDS 配置。

```bash
cd /home/nan097/ros2_arm
bash tools/build_simulation.sh
bash tools/run_sort_demo.sh
```

第二条命令打开 Gazebo、RViz，等待机器人就绪后自动执行一次真实仿真任务。终端输出 `[SORT]` 阶段反馈和任务结果；成功后界面保留，红箱在放置工位，机器人回到原点。Ctrl+C 结束本次启动。

只启动系统、不自动发送任务：

```bash
bash tools/run_sort_simulation.sh
```

其他终端发送任务时需使用相同环境与域：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source tools/simulation_env.sh
export ROS_DOMAIN_ID=75
ros2 action send_goal /task/execute_sort \
  landerpi_task_interfaces/action/ExecuteSortTask \
  '{task_id: manual_1, object_class: red_box, pick_station: pick, place_station: place, quantity: 1, return_home: true}' \
  --feedback
```

无界面：`bash tools/run_sort_demo.sh gui:=false rviz:=false`。ROS 域默认 75；Gazebo 每次自动选择空闲端口。启动文件支持 `ros_domain_id`、`gazebo_port`、`csv_path` 参数。不要在同一个 ROS 域同时启动 dry_run 任务服务器和此完整仿真服务器。

## 接口

| Action | 仿真实现 |
|---|---|
| `/task/execute_sort` | 单红箱完整任务；quantity=1，pick→place；状态反馈、子动作取消、480 s 总超时、CSV |
| `/navigation/navigate_station` | Nav2 NavigateToPose，工位 home/pick/place，到站后低速精定位 |
| `/fine_alignment/align` | mode=1 通过仿真 odom 修正工位位姿，接受请求容差/超时；mode=0 跳过；其他模式明确拒绝 |
| `/perception/detect_object` | 真实 Gazebo RGB-D 图像、HSV 红色轮廓、有效深度中值、CameraInfo 投影、TF |
| `/manipulation/move_to_joint` | 五个机械臂关节的 MoveIt 规划与可选执行 |
| `/manipulation/move_to_pose` | arm 组位置 IK、碰撞规划与可选执行 |
| `/manipulation/move_named_pose` | home/transport/observe，当前仿真均采用安全直立姿态；支持 execute=false |
| `/manipulation/pick_object` | 接近、闭合夹爪、物理附着、MoveIt 附着、抬升反馈检查、运输位 |
| `/manipulation/place_object` | 下降、打开夹爪、物理及场景解绑、退出、箱体实际落点检查 |

`/robot/status` 由真实关节、odom、雷达、RGB-D、TF、Nav2 和 MoveIt 服务就绪状态生成。`/robot/reset_faults` 清除任务故障；运行中或仍抓着箱体时拒绝复位。取消后确认停止并复位故障才能重发任务。

标准 Action 类型来自 `landerpi_task_interfaces`；仿真适配集中于 `landerpi_simulation/scripts/sort_runtime.py`。其他上层包的实物后端仍保持框架状态，不应误认为已完成硬件迁移。

## 仿真模型边界

- 实际 URDF 为五个机械臂转动关节加一个夹爪控制关节。采用 position-only IK，不宣称任意六自由度末端姿态能力。
- 底盘采用理想全向速度模型；`map→odom` 为恒等变换，odom 来自 Gazebo 真值。未使用 AMCL、SLAM 或真实轮速里程计。
- 抓取工位使用固定俯视 RGB-D 相机。物体三维位置来自图像与深度；箱体尺寸为配置已知的 40×40×50 mm。只支持直立红箱，未实现复杂 6D 姿态识别。
- Gazebo 中的理想抓取由真实 fixed joint 连接工具与箱体。必须夹爪闭合且工具与箱体中心距离 ≤30 mm 才能附着。机器人与箱体的接触冲量被排除，桌面、地面和释放后的重力仍由物理引擎计算。不能用这些结果推断真实摩擦抓取成功率。
- 机械臂动作时使用仿真驻车 fixed joint，导航前完全销毁驻车约束。无移动中操作或底盘与机械臂联合规划。
- MoveIt 使用简化碰撞盒和真实关节限位。只允许目标接触夹爪相关链接，以及抓取/放置瞬间接触相应支撑桌面；抬升/释放后恢复桌面碰撞检查。移动底盘停下后刷新场景坐标。
- `GetEntityState` 用于附着/运输/落点验证；任务正常运行不调用 SetEntityState。后者只在验收脚本中重置初始摆放或注入无目标故障。
- 导航失败最多重试一次，检测最多三次；附着之后的操作失败进入故障状态，不自动释放箱体。

## 验收

```bash
# 单轮完整功能验证
bash tools/run_sort_test.sh

# 文档建议的基线：10 次完整任务 + 10 次独立视觉抓放，共 20 次抓取
bash tools/run_sort_test.sh --runs 10 --grasp-runs 10 --fault-check
```

脚本先检查驻车锁止/释放与仅规划模式，然后运行任务。独立读取 Gazebo 箱体落点、附着状态、返航 odom、关节反馈和 MoveIt Planning Scene，检查解除附着、恢复桌面碰撞规则，并核对任务 CSV。`--fault-check` 覆盖取消、互斥、非法数量、无目标、有限重试和复位。

默认生产任务 CSV：`/tmp/landerpi_experiments/simulation_tasks_v2.csv`，包含执行模式、软件版本、结果码、阶段耗时和完成数量。验收脚本为每次进程生成独立 CSV 和日志路径，并在终端打印路径。

Gazebo Classic 和当前 MoveIt 属于环境已有版本。MoveIt 退出时仍可能出现已有 class_loader 析构告警/崩溃；运行阶段的动作结果和物理验收另行记录，不用关闭日志替代功能验收。

放置动作将箱体移至目标中心上方 4 cm，再解除固定抓取约束，让箱体在重力下落到桌面；随后核对实际箱体位置。此策略避免五轴末端姿态变化造成桌面穿透。

动态 `odom → base_footprint` 由 scan_filter 节点使用 /odom 的原始位姿和时间戳发布，Gazebo planar_move 的重复 TF 发布已关闭。验收日志并列记录里程计和 TF 坐标。

仿真脚本默认使用工作区中的 Cyclone DDS，配置本机发现及足够的参与者索引。长时间组合测试曾出现 Nav2 内部 TF 缓存不更新，单独切换 DDS 未解决。控制器现在拒绝超过 0.5 s 的过期 TF，并在代价地图互斥锁内进行碰撞检查。雷达消息保留原始测量时间戳，等待未来 50 ms 的 TF 已存在，再延迟至测量后 200 ms 发布，减少 Nav2 异步 TF 等待链路阻塞。完整基线结果以验证记录为准。

雷达等待时序的依据：[Nav2 Humble ObstacleLayer 源码](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/plugins/obstacle_layer.cpp)。Humble 的 LaserScan 分支固定要求测量时间之后 50 ms 的 TF；本项目在消息进入 Nav2 前等待该变换就绪，不篡改测量时间。

需要让图形预览独立于命令会话保留时，可运行 `python3 tools/start_sort_preview.py`，它使用域 76，并将日志写入 `/tmp/landerpi_sort_visible.log`。再次调用会检查已有预览；用 `python3 tools/stop_sort_preview.py` 结束该预览。手动查询此预览时使用 ROS_DOMAIN_ID=76；常规前台入口仍默认域 75。
