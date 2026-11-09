# 基础框架验证记录

日期：2026-10-01。环境：ROS 2 Humble；模式：dry_run。当前目录不是有效 Git 仓库，未生成 commit 标识。

| 检查 | 结果 |
|---|---|
| colcon list | 22 个活动包，无重复包名，archive 已排除 |
| 新增基础框架构建 | 11 个包全部通过 |
| ros2 interface show ExecuteSortTask | 生成类型可读取 |
| task_system.launch.py --show-args | 解析成功，默认 execution_mode=dry_run |
| tools/test_framework.py | 5 项通过：包/语法、接口逐字契约、循环/返航顺序、状态重置、无效深度拒绝 |
| tools/test_task_action.py | 真实 ROS Action 通信测试通过：两件物体、返航一次、并发拒绝、重复任务、取消、超时、live 拒绝、非法数量、CSV |

测试初次运行发现 rclpy.Node 内置属性名称冲突及返航序列边界问题，已修正并回归通过。DDS 套接字被沙箱限制，Action 测试经授权在沙箱外运行。

未验证：旧底层全部包的完整构建、硬件在线、实际 TF、Nav2、MoveIt、相机、舵机轨迹与真实物流闭环。构建成功不代表骨架模块已完成算法或硬件集成。

复现命令见根目录 README 和 tools/ 两个测试脚本。下一步先补齐接口盘点的运行时证据，再开发一个可单独验收的 Adapter。

用户明确先跑仿真后，新增第 12 个基础包 landerpi_simulation，活动包共 23 个。Gazebo 仿真验证范围另见 simulation_test_report.md；本报告的 dry_run 检查不代表真实仿真闭环。
