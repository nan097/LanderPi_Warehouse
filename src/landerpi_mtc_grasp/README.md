# RGB-D 视觉抓放

使用随臂 RGB-D 相机检测指定颜色物块，通过标准 MoveIt Task Constructor 阶段完成观察、接近、闭爪、抬升、左侧地面放置和归位。机械臂通过直立运输姿态连接抓取与放置。

```bash
bash tools/run_mtc_color_grasp.sh target_color:=red
```

颜色支持 `red`、`green`、`blue`，无界面添加 `gui:=false rviz:=false`。安装与构建见[根目录 README](../../README.md)。

`color_vision.py` 使用 RGB、深度、相机内参与图像时间戳对应的 TF 得到目标位置；`color_grasp.cpp` 使用 MTC 的 CurrentState、MoveTo 和 ModifyPlanningScene 规划执行。仿真分别维护 MoveIt 附着模型与 Gazebo 物理附着，并检查实际抬升及落点。

## 实机入口

```bash
bash tools/run_mtc_color_grasp_hardware.sh target_color:=red
```

需要 LanderPi 板卡、真实舵机回读、Aurora930 相机驱动、`/dev/rrc` 权限与正确标定。START 请求抓放，BACK 取消；反馈或手柄心跳丢失会停止任务。任务期间底盘保持停止。默认速度和加速度比例均为 0.2。

实机入口尚未通过现场硬件验收。`execution_complete=true grasp_verified=false` 仅表示轨迹执行完成，不能确认物体被夹住或已经落地。

## 几何与接口检查

```bash
python3 -m unittest discover -s src/landerpi_mtc_grasp/tests -v
python3 tools/test_mtc_hardware_interfaces.py
```

虚拟设备检查不替代实机验收。机器人使用五轴位置 IK；仿真为理想刚性抓取，不能推断实物抓取成功率。
