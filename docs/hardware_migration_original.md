# LanderPi ROS 2 硬件移植包

本目录从原项目提取用于**保留原 STM32 控制板**的 ROS 2 源码。目录中的 `src/` 可作为 colcon 工作区源码使用。原仓库文件未被移动或删除。

## 包含内容

| 路径 | 用途 |
| --- | --- |
| `src/driver/ros_robot_controller`、`ros_robot_controller_msgs` | `/dev/rrc` 串口协议、STM32 节点及消息/服务 |
| `src/driver/controller` | 底盘运动学、里程计与初始位姿 |
| `src/driver/servo_controller`、`servo_controller_msgs` | 机械臂总线舵机控制与消息 |
| `src/driver/kinematics`、`kinematics_msgs` | 机械臂运动学及消息 |
| `src/driver/sdk` | 通用辅助代码；其中 `sdk/led.py` 依赖 Jetson.GPIO |
| `src/peripherals` | 雷达、相机、手柄、IMU 相关启动文件、参数和设备规则 |
| `src/simulations/landerpi_description` | URDF、TF 和机器人描述，底盘启动依赖它 |
| `src/xf_mic_asr_offline`、`xf_mic_asr_offline_msgs` | 可选的语音硬件支持 |
| `src/bringup` | 原项目总启动文件，仅供参考；依赖未纳入的应用与服务 |
| `launch/core_hardware.launch.py` | 此移植包新增的基础硬件启动入口 |
| `launch/all_hardware.launch.py` | 按参数启用雷达、相机、手柄、麦克风及 IMU 滤波 |
| `src/calibration` | IMU 滤波启动所需校准文件与节点 |

## 部署到 Linux ROS 2 主机

1. 将本目录复制到 Linux 主机，例如 `~/landerpi_ws`。安装与目标 ROS 2 发行版匹配的依赖。至少需要 `python3-serial`、`python3-yaml`、`python3-numpy`、`python3-scipy`、`python3-opencv`、`robot-localization`、`imu-filter-madgwick`、`xacro`、`robot-state-publisher`；启用雷达、相机、语音时还需要对应厂商驱动和 ROS 包。具体依赖以目标 ROS 2 版本和 `colcon build` 输出为准。
2. 核对 `src/peripherals/scripts/` 与 `src/driver/ros_robot_controller/scripts/` 下的 udev 规则。原控制板使用 `/dev/rrc`（`ttyACM*`，**1,000,000 baud**）；部分雷达和麦克风规则按原机器 USB 物理端口匹配，换主机后需重写匹配条件。安装规则后重新加载 udev 或重插设备。
3. 在工作区根目录执行：

   ```bash
   source /opt/ros/<发行版>/setup.bash
   colcon build --symlink-install
   source install/setup.bash
   ls -l /dev/rrc
   ros2 launch launch/core_hardware.launch.py
   ```

   按实际安装的硬件逐项启用，例如：

   ```bash
   ros2 launch launch/all_hardware.launch.py lidar:=ld19 camera:=usb joystick:=true mic:=true imu_filter:=true
   ```

   `lidar` 可选 `none/ld19/ms200/a1/g4`；`camera` 可选 `none/usb/ascamera/aurora`。默认只启动 STM32 控制板。`mic:=true` 只启动麦克风唤醒串口节点；完整离线语音识别仍需配置原 `mic_init.launch.py` 的 `MIC_TYPE`、`ASR_LANGUAGE` 和资源路径。外设型号不同，须安装对应厂商驱动；`imu_filter` 另需 `imu_calib` 和 `imu_complementary_filter` ROS 包。

   也可以先用 `ros2 launch ros_robot_controller ros_robot_controller.launch.py` 单独验证控制板串口与 IMU、电池等数据，再接入底盘和舵机。

## 按需启用外设

- 雷达：设置 `LIDAR_TYPE` 并检查 `src/peripherals/launch/lidar.launch.py` 对型号的分支，以及 `/dev/lidar` 或 `/dev/ldlidar` 和波特率。原文件的部分型号分支实际共用 LD19/MS200 启动文件，接入 A1/G4 时应直接检查对应 `launch/include/` 文件。
- 相机：设置 `DEPTH_CAMERA_TYPE`；USB 摄像头默认 `/dev/video0`，深度相机需要匹配 `ascamera`/`aurora` 驱动及其外部配置目录。
- 手柄：默认 `/dev/input/js0`。语音阵列默认 `/dev/ring_mic`，115200 baud。

## 硬件接口移植核对表

| 接口 | 已复制的入口 | 目标系统需核对 |
| --- | --- | --- |
| STM32 USB 串口 | `ros_robot_controller_sdk.py`、`99-ttyACM0.rules` | `/dev/rrc`、1000000 baud、权限；控制板固件与协议保持一致 |
| 电机/PWM 舵机/总线舵机/LED/蜂鸣器/RGB/OLED | `ros_robot_controller_node.py`、`ros_robot_controller_msgs` | 接线、舵机 ID、零位、电机方向；均经 STM32 串口访问 |
| 控制板 IMU/按键/电池/手柄/SBUS | 同上 | ROS 话题数据是否正确；IMU 滤波可选 |
| 激光雷达 | `peripherals/launch/include/`、`lidar.rules` | 实际型号、串口路径及 115200/230400 baud、厂商 ROS 驱动 |
| USB/深度相机 | `peripherals/launch/`、`usb_cam_param.yaml`、相机 udev 规则 | `/dev/video0` 或相机 USB 设备、厂商驱动和配置目录 |
| USB 游戏手柄 | `joystick_control.py`、对应 launch | `/dev/input/js0` 与 `joy` ROS 包 |
| 语音阵列 | `xf_mic_asr_offline`、`xf_mic.rules` | `/dev/ring_mic`、115200 baud、所带 `.so` 与 CPU 架构兼容 |
| 主机 GPIO LED | `driver/sdk/sdk/led.py` | 仅 Jetson.GPIO，BCM 24；其他主机需替换 GPIO 实现 |

仓库中没有控制板电机端子、舵机端子的电压或物理针脚定义；这些必须以控制板硬件资料核对。仓库也没有主机直连 I²C、SPI 或 CAN 的驱动实现。

## 必须复核的原项目配置

- `MACHINE_TYPE`、`need_compile`、`LIDAR_TYPE`、`DEPTH_CAMERA_TYPE` 等环境变量在多个原始启动文件中直接读取。使用原启动链前须设置；新增的 `core_hardware.launch.py` 不依赖这些环境变量。
- 若运行原 `bringup.launch.py`，还要补齐原项目的 `app`、`rosbridge_server`、`web_video_server` 和相关视觉/导航依赖。它并非本包的基础启动入口。
- 多处源码固定引用 `/home/ubuntu/ros2_ws`、`/home/ubuntu/software` 或 `/home/ubuntu/third_party_ros2`；迁移后应改为实际路径或 ROS 包资源路径。尤其核对舵机配置、动作组、底盘校准和相机配置。
- `src/driver/kinematics/kinematics/` 中的 `.so` 是预编译二进制。更换 CPU 架构或 Python ABI 后需要取得兼容版本或重新编译。
- 底盘运动学、轮径/轮距、舵机零位、机械臂 URDF 与实物一致后才能安全运行。先架空车轮、低速验证方向及急停。

## 文件来源与范围

复制自原仓库的上述 ROS 2 包，未修改其内部实现。未纳入演示、导航、SLAM、AI 视觉、App 等上层功能；如需要这些功能，可在基础硬件验证通过后从原仓库按依赖追加。
