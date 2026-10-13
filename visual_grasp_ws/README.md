# Visual Grasp 视觉抓取

## 1. 项目介绍

`visual_grasp_ws` 是面向 PiPER-H 机械臂和 Intel RealSense D435 的
ROS 2 传统视觉抓取工作区，目标为识别并抓取 30 mm 红、绿、蓝木质方块。

功能包括：

- AprilTag 眼在手上（eye-in-hand）标定。
- D435 彩色图与对齐深度融合的三色方块检测。
- 预抓取、抓取、抬升和空中回正四段位姿规划。
- 带反馈、超时、工作空间和夹爪状态检查的 PiPER 抓取执行器。
- 人工确认的两阶段抓取，以及显式调用的完整抓取。

```text
D435 RGB + aligned depth -> color cube detector -> block pose
                                                    |
hand-eye TF + block pose -> grasp planner -> four-stage TCP poses
                                                    |
arm/gripper feedback <-> guarded PiPER executor -> pick result
```

> 安全约束：默认 `allow_robot_commands:=false`，只做检测和规划，不向机械臂
> 发送运动指令。执行器使用驱动的 MoveP/MoveL/MoveJ，不是 MoveIt 2 全场景
> 碰撞规划。首次实机运行必须低速、清空工作空间并准备实体急停。

若该模块用于完整导航抓取任务，整体顺序仍然是：先使用 SLAM 建图并保存
地图，再启动 Nav2；Nav2 启动后必须在 RViz 使用 **2D Pose Estimate** 设置
正确初始位姿，最后才能启动导航—抓取—返航状态机。

## 2. 硬件配置与运行环境

默认硬件：

| 类别 | 默认配置 |
| --- | --- |
| 机械臂 | PiPER-H，AGX ROS 2 驱动 |
| 机械臂总线 | SocketCAN `can0` |
| 夹爪 | `agx_gripper`，实时宽度和故障反馈 |
| RGB-D 相机 | Intel RealSense D435，眼在手上安装 |
| 标定板 | AprilTag `tag36h11`，ID 3，tag 黑框实测边长 0.035 m |
| 抓取物 | 30 mm 红、绿、蓝方块 |
| TCP | `piper_link6 -> piper_tcp_link`，名义法兰 Z 轴 0.142 m |

软件环境：

- Ubuntu 22.04
- ROS 2 Humble / Python 3.10
- RealSense ROS 2 驱动
- PiPER/AGX 机械臂驱动及 `agx_arm_msgs`
- OpenCV、NumPy、cv_bridge、message_filters、tf2_ros、RViz2

主要坐标系：

```text
map -> odom -> base_link -> piper_base_link
                             -> piper_link1 ... piper_link6
                                                    |-> piper_tcp_link
                                                    `-> camera_link
                                                         `-> camera_color_optical_frame
```

手眼标定求解 `piper_link6 -> camera_color_optical_frame`，再利用 D435 内部 TF
换算并保存 `piper_link6 -> camera_link`。

## 3. 运行环境相关包与依赖安装

以下命令默认在仓库根目录 `nav-grasp-robot/` 执行。

安装基础工具，拉取 AGX 驱动源码，再使用 rosdep 安装 ROS 依赖：

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
vcs import . < dependencies.repos
rosdep install --from-paths visual_grasp_ws/src \
  --ignore-src --rosdistro humble -r -y
```

D435 和 PiPER/AGX 的底层驱动需先按硬件厂商说明单独验证。

复制本机配置：

```bash
cp config/project.env.example config/project.env
```

编辑 `config/project.env`，至少配置：

| 变量 | 用途 |
| --- | --- |
| `ROS_SETUP` | ROS 2 Humble 环境 |
| `ARM_DRIVER_SETUP` | AGX 驱动 overlay |
| `ARM_DRIVER_WS` | AGX 驱动工作区 |
| `ARM_CONTROL_SCRIPT` | 机械臂控制启动脚本 |
| `CAMERA_START_SCRIPT` | D435 启动脚本 |
| `CAMERA_STOP_SCRIPT` | D435 停止脚本 |
| `CAN_SETUP_SCRIPT` | 机械臂 CAN 初始化脚本 |
| `NAV_GRASP_CALIBRATION_DIR` | 本机手眼标定结果目录 |
| `NAV_GRASP_GRIPPER_REPORT` | 本机夹爪手动行程报告 |

编译：

```bash
./visual_grasp_ws/build.sh
```

加载工作区：

```bash
source /opt/ros/humble/setup.bash
source visual_grasp_ws/install/setup.bash
```

## 4. 正常运行命令

### 4.1 启动相机和机械臂驱动

终端 1，启动 D435：

```bash
source scripts/project_env.bash
"$CAMERA_START_SCRIPT"
```

终端 2，启动机械臂驱动：

```bash
./visual_grasp_ws/start_arm_control.sh --replace
```

`--replace` 用于安全清理旧驱动实例并启动新实例。首次测试前应确认：

```bash
source scripts/project_env.bash
source "$ROS_SETUP"
source "$ARM_DRIVER_SETUP"
ros2 param get /arm/agx_arm_ctrl_single_node tcp_offset
ros2 param get /arm/agx_arm_ctrl_single_node speed_percent
ros2 param get /arm/agx_arm_ctrl_single_node effector_type
```

AGX 驱动的 `tcp_offset` 应保持全零，避免与本工程 0.142 m TCP 发生双重补偿；
首次测试速度不高于 10%，末端类型应为 `agx_gripper`。

### 4.2 首次手眼标定

固定标定板，然后启动标定节点和交互采样器：

```bash
./visual_grasp_ws/start_handeye_collection.sh \
  arm_pose_topic:=/arm/feedback/tcp_pose
```

每次先移动机械臂，等待机械臂完全停止且标签完整可见，再在采样终端按回车。
建议采集 20--30 个同时改变平移与腕部方向的姿态。

```text
回车    采集一组
s       求解并保存
r       清空样本（需确认）
h       显示帮助
q       退出
```

样本和结果写入 `NAV_GRASP_CALIBRATION_DIR`；未设置该变量时默认使用
`~/.config/nav_grasp/calibration/`。它们属于本机实测数据，不应提交到 GitHub。

### 4.3 只读检测和规划

终端 3，以默认禁止机械臂指令的模式启动感知、TF、规划和执行器接口：

```bash
./visual_grasp_ws/start_perception.sh
```

该命令会启动 `visual_grasp_bringup/perception.launch.py`，但因
`allow_robot_commands:=false` 不会让机械臂运动。

请求一次检测和抓取候选规划：

```bash
ros2 service call /visual_grasp/plan_pick std_srvs/srv/Trigger '{}'
ros2 topic echo /visual_grasp/pick_sequence --once
```

启动 RViz2 可视化：

```bash
./visual_grasp_ws/start_rviz.sh
```

确认点云、方块位置、TCP 方向以及预抓取/抓取/抬升/回正四段位姿合理。

### 4.4 两阶段实机抓取

先在只读感知终端按 `Ctrl+C`，确认节点完全退出，再显式允许实机指令：

```bash
./visual_grasp_ws/start_perception.sh allow_robot_commands:=true
```

阶段一，生成最新规划并只移动到方块上方：

```bash
ros2 service call /visual_grasp/plan_pick std_srvs/srv/Trigger '{}'
ros2 service call /visual_grasp/move_to_pregrasp std_srvs/srv/Trigger '{}'
ros2 service call /visual_grasp/pick_status std_srvs/srv/Trigger '{}'
```

现场确认 TCP 位于方块正上方、夹爪方向正确、垂直下降路径无障碍后，
执行阶段二：

```bash
ros2 service call /visual_grasp/execute_pick std_srvs/srv/Trigger '{}'
```

完成或排除故障后复位抓取状态：

```bash
ros2 service call /visual_grasp/reset_pick std_srvs/srv/Trigger '{}'
```

紧急情况优先使用实体急停。软件急停命令：

```bash
ros2 service call /arm/emergency_stop std_srvs/srv/Empty '{}'
```

## 5. 一键运行命令

### 5.1 一键启动视觉抓取栈

相机和机械臂驱动已经启动、手眼标定已完成时，一条命令启动检测、
规划、执行器接口和 TF：

```bash
./visual_grasp_ws/start_perception.sh
```

这是安全的只读模式。已完成低速分阶段验证后，才使用：

```bash
./visual_grasp_ws/start_perception.sh allow_robot_commands:=true
```

### 5.2 一键计划并执行一次完整抓取

在允许实机指令的感知栈已启动、目标完整可见且现场安全时：

```bash
./visual_grasp_ws/pick_once.sh
```

`pick_once.sh` 会先确认机械臂返回 CAN 控制模式，再调用
`/visual_grasp/plan_pick`，规划成功后调用 `/visual_grasp/execute_full_pick`。
该命令会产生真实机械臂运动，不用于首次联调。

需要同时启动导航、相机、机械臂和状态机时，使用总项目入口：

```bash
./nav_grasp_ws/start_full_mission.sh \
  --map /absolute/path/to/map.yaml \
  --mission-config /absolute/path/to/mission.local.yaml
```

## 6. 输出、检查与故障处理

主要输出：

| 名称 | 作用 |
| --- | --- |
| `/visual_grasp/blocks` | `piper_base_link` 坐标系下的方块检测 |
| `/visual_grasp/debug_image` | HSV 分割和深度检查图 |
| `/visual_grasp/block_markers` | RViz 3D 方块标记 |
| `/visual_grasp/pick_sequence` | 预抓取、抓取、抬升、回正位姿 |
| `/visual_grasp/execution_state` | 抓取执行器状态 |
| `/visual_grasp/commanded_flange_pose` | TCP 补偿后的法兰目标，用于检查 |

### 检测不到方块

- 检查 D435 彩色图、对齐深度和相机 TF。
- 通过 `/visual_grasp/debug_image` 检查光照和 HSV 阈值。
- 检查 `target_color` 是 `red`、`green`、`blue` 或任意颜色。

### `OPENING timed out` 或夹爪零位异常

停止视觉抓取和 AGX 驱动，确认两指之间无物体，然后按提示校准：

```bash
./visual_grasp_ws/calibrate_gripper.sh
```

仅测量夹爪手动开合端点时：

```bash
./visual_grasp_ws/measure_gripper_range.sh
```

不得伪造 `homing_status`。如实机固件持续报告未回零，只能在完成手动行程
报告验证后，通过本机 profile 显式允许受限降级运行。

建议验收顺序：

1. RViz 检查 TCP 方向和 0.142 m 名义长度。
2. 检查 AprilTag 坐标轴和重投影误差。
3. 使用未参与求解的姿态验证固定标签在 `piper_base_link` 下不漂移。
4. 禁止控制模式下检查四段位姿。
5. 低速执行阶段一，只验证夹爪是否到达方块正上方。
6. 人工确认后再执行阶段二。

## 7. 作者

视觉抓取及 `visual_grasp_ws/` 由
[@GGbond61](https://github.com/GGbond61) 独立完成和维护。

贡献者列表见 [`../CONTRIBUTORS.md`](../CONTRIBUTORS.md)，协作规则见
[`../CONTRIBUTING.md`](../CONTRIBUTING.md)。
