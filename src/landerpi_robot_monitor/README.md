# landerpi_robot_monitor

Purpose: 上层任务层基础框架，接口遵循根目录设计文档。

Status: 基础骨架；硬件和算法集成尚未实现。

Subscriptions / Publications / Actions / Services / Parameters / TF:
见 [框架接口清单](../../docs/framework_status.md)。未列出的接口尚未实现。

Dependencies: 见 package.xml。

Build / Test:
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to landerpi_robot_monitor --symlink-install
source install/setup.bash
```

Acceptance: 构建成功；launch 可解析；未集成模块不会发布运动指令。
Failure Behavior: 未配置硬件接口不进入就绪状态，不报告真实运动成功。
