# 导航—抓取—返航状态机

## 1. 项目介绍

`nav_grasp_real/src/nav_grasp` 是 ROS 2 Humble 任务编排工作区。`nav_grasp_mission` 使用
YASMIN 调用已运行的 Nav2 和 `nav_grasp_real/src/visual_grasp` 接口，完成导航—抓取—返航。

```text
检查依赖 -> 记录返航起点 -> 导航到抓取点 -> 等待底盘静止
        -> 检测目标并生成抓取计划 -> 执行抓取
        -> 等待抓取成功 -> 返回本次记录的起点
```

状态机只负责任务编排，不实现导航、感知、抓取规划或机械臂驱动；
具体算法由 `nav_grasp_real/src/slam` 和 `nav_grasp_real/src/visual_grasp` 提供。

任务运行前必须满足：

1. 已先使用 SLAM 完成建图并保存地图。
2. Nav2 已加载该地图。
3. 已在 RViz 使用 **2D Pose Estimate** 设置正确初始位姿。
4. 手眼标定、抓取停车点和机械臂工作空间已完成验证。

> 安全提示：此流程会驱动真实底盘和机械臂。开始前必须确认实体急停
> 可用，并清空导航路线、抓取区和机械臂工作空间。实体危险时优先使用
> 实体急停，不要仅依赖软件取消。

## 2. 硬件配置与运行环境

完整任务使用的默认硬件：

| 类别 | 默认配置 |
| --- | --- |
| 移动底盘 | AgileX Tracer，SocketCAN 接口名和波特率按本机配置 |
| 雷达 | Livox MID-360，提供 `/scan` |
| 相机 | Intel RealSense D435 |
| 机械臂 | PiPER-H / AGX 驱动，`can0` |
| 夹爪 | AGX 夹爪 |
| 计算机 | Ubuntu 22.04 图形桌面 |

软件环境：

- ROS 2 Humble / Python 3.10
- Nav2、YASMIN、YASMIN ROS
- `nav_grasp_real/src/slam`、`nav_grasp_real/src/visual_grasp` 和 `nav_grasp_real/src/nav_grasp` 均已构建
- `gnome-terminal`、`flock`、`pgrep`、SocketCAN 工具
- Tracer、Livox、D435 和 PiPER/AGX 驱动可用

状态机依赖的主要 ROS 接口：

| 接口 | 用途 |
| --- | --- |
| `/navigate_to_pose` | Nav2 导航 action |
| `/visual_grasp/plan_pick` | 目标检测与抓取规划 |
| `/visual_grasp/execute_full_pick` | 完整抓取执行 |
| `/visual_grasp/execution_state` | 抓取执行状态 |
| `/odom` | 底盘静止与新鲜度检查 |
| `map -> base_link` | 记录返航起点 |

## 3. 运行环境相关包与依赖安装

以下命令默认在仓库根目录 `nav-grasp-robot/` 执行。

安装 YASMIN 和构建工具：

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep \
  ros-humble-yasmin ros-humble-yasmin-ros
```

安装状态机包依赖：

```bash
rosdep install --from-paths nav_grasp_real/src/nav_grasp/src \
  --ignore-src --rosdistro humble -r -y
```

复制本机配置：

```bash
cp nav_grasp_real/project.env.example nav_grasp_real/project.env
```

一键启动器至少使用：

| 变量 | 用途 |
| --- | --- |
| `ROS_SETUP` | ROS 2 Humble 环境 |
| `ARM_DRIVER_SETUP` / `ARM_DRIVER_WS` | 机械臂 overlay 与工作区 |
| `ARM_CONTROL_SCRIPT` | 机械臂启动脚本 |
| `CAMERA_START_SCRIPT` / `CAMERA_STOP_SCRIPT` | D435 启停脚本 |
| `CAN_SETUP_SCRIPT` | 机械臂 CAN 初始化 |
| `BASE_CAN_INTERFACE` / `BASE_CAN_BITRATE` | 底盘 CAN 配置 |
| `NAV_GRASP_MAP` | Nav2 地图 YAML 路径 |
| `NAV_GRASP_MISSION_CONFIG` | 任务参数路径 |

配置抓取停车点：

```bash
cp nav_grasp_real/src/nav_grasp/src/nav_grasp_mission/config/mission.yaml \
   nav_grasp_real/src/nav_grasp/src/nav_grasp_mission/config/mission.local.yaml
```

编辑 `mission.local.yaml`：

```yaml
nav_grasp_mission:
  ros__parameters:
    target_pose_configured: true
    target_pose_xyyaw: [x, y, yaw]
```

`x`、`y` 是 `map` 坐标系下的米制坐标，`yaw` 是弧度。该位置必须让目标
进入相机视野和机械臂工作空间。`mission.local.yaml` 已被 Git 忽略。

构建全部工作区：

```bash
./scripts/build_all.sh
```

仅重新构建状态机：

```bash
./nav_grasp_real/src/nav_grasp/build.sh
```

## 4. 正常运行命令

本节用于分项调试。每个子系统应在独立终端中运行。

### 4.1 启动导航

必须先用 SLAM 完成建图并保存地图。按
[`../slam/README.md`](../slam/README.md) 启动底盘、雷达和 `/scan`，
然后启动 Nav2：

```bash
source nav_grasp_real/src/slam/setup.bash
ros2 launch nav navigation.launch.py \
  map:="/absolute/path/to/my_map.yaml"
```

Nav2/RViz 启动后，必须先使用 **2D Pose Estimate** 设置机器人的真实初始
位置和朝向，确认激光与地图重合。状态机会使用当时的位姿作为返航点。

### 4.2 启动相机、机械臂和视觉抓取

启动 D435：

```bash
source scripts/project_env.bash
"$CAMERA_START_SCRIPT"
```

启动机械臂：

```bash
./nav_grasp_real/src/visual_grasp/start_arm_control.sh --replace
```

状态机需要调用真实抓取，因此在已完成手眼标定和分阶段低速验证后启动：

```bash
./nav_grasp_real/src/visual_grasp/start_perception.sh allow_robot_commands:=true
```

视觉抓取的标定和安全验证见
[`../visual_grasp/README.md`](../visual_grasp/README.md)。

### 4.3 启动状态机

```bash
source /opt/ros/humble/setup.bash
source nav_grasp_real/src/nav_grasp/install/setup.bash
ros2 launch nav_grasp_mission mission.launch.py \
  params_file:="$PWD/nav_grasp_real/src/nav_grasp/src/nav_grasp_mission/config/mission.local.yaml"
```

状态机节点启动本身不会移动机器人。先检查依赖和定位：

```bash
ros2 action list | grep /navigate_to_pose
ros2 service list | grep -E '/visual_grasp/plan_pick|/visual_grasp/execute_full_pick'
ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map base_link
```

确认 RViz 初始位姿、导航路线、抓取区和机械臂工作空间安全后，
显式启动任务：

```bash
ros2 service call /nav_grasp/start std_srvs/srv/Trigger '{}'
```

查看运行状态：

```bash
ros2 service call /nav_grasp/status std_srvs/srv/Trigger '{}'
ros2 topic echo /nav_grasp/state
ros2 topic echo /nav_grasp/event
```

## 5. 一键运行命令

完成地图、目标位姿、本机环境和硬件启停脚本配置后运行：

```bash
./nav_grasp_real/src/nav_grasp/start_full_mission.sh \
  --map /absolute/path/to/map.yaml \
  --mission-config /absolute/path/to/mission.local.yaml
```

启动器会依次打开：

1. 底盘与雷达，等待 `/odom` 和 `/scan`。
2. D435 相机，等待彩色图像。
3. Nav2 与 RViz，等待 `/navigate_to_pose`。
4. 机械臂驱动，等待机械臂和夹爪反馈话题。
5. 允许实机指令的视觉抓取栈。
6. 任务状态机。

所有组件就绪后，它会停在主控终端等待。此时必须：

1. 在 RViz 使用 **2D Pose Estimate** 设置初始位姿。
2. 确认 LaserScan 与地图重合。
3. 确认当前位置适合作为返航终点。
4. 清空导航路线、抓取区和机械臂工作空间。
5. 回到主控终端按回车。

未检测到 `map -> base_link` 时，启动器不会提交任务。

## 6. 状态机运行顺序

| 状态 | 作用 |
| --- | --- |
| `WAIT_DEPENDENCIES` | 检查 Nav2、抓取 service、TF、里程计和抓取状态 |
| `CAPTURE_START_POSE` | 记录当前 `map -> base_link` 位姿作为返航点 |
| `NAVIGATE_TO_TARGET` | 向 Nav2 发送抓取停车点 |
| `WAIT_BASE_STILL` | 要求底盘在新鲜里程计下连续静止 |
| `PLAN_PICK` | 在默认 15 秒窗口内重试检测和抓取规划 |
| `EXECUTE_PICK` | 调用完整抓取执行接口 |
| `WAIT_PICK_RESULT` | 等待抓取执行器返回 `SUCCEEDED` 或 `ERROR` |
| `RETURN_TO_START` | 仅在抓取成功后返回记录的起点 |

终止状态是 `DONE`、`ERROR` 或 `CANCELED`。任何步骤报错或超时都不会继续
后续动作；只有抓取执行器返回 `SUCCEEDED` 才会返航。

## 7. 取消、退出与故障检查

取消任务：

```bash
ros2 service call /nav_grasp/cancel std_srvs/srv/Trigger '{}'
```

导航阶段取消会请求 Nav2 取消当前目标并发布底盘零速度。抓取阶段取消
不会中断已被机械臂接受的动作，也不会开始返航。紧急情况使用实体急停。

任务结束、报错或取消后，先检查现场和各子系统，再恢复为 `IDLE`：

```bash
ros2 service call /nav_grasp/reset std_srvs/srv/Trigger '{}'
```

`reset` 只清除状态机状态，不复位、松开或移动机械臂。

一键启动器运行时，应在主控终端按 `Ctrl+C`。它会先取消任务、发布底盘
零速度，必要时尝试机械臂软件急停，然后逆序关闭本次启动的进程。

常见检查：

```bash
ros2 action list | grep /navigate_to_pose
ros2 service list | grep -E '/visual_grasp|/nav_grasp'
ros2 topic hz /odom
ros2 topic echo /visual_grasp/execution_state
ros2 run tf2_ros tf2_echo map base_link
```

- 停在 `WAIT_DEPENDENCIES`：检查 Nav2 action、视觉抓取 service、`/odom` 和 TF。
- 停在 `PLAN_PICK`：检查目标视野、颜色配置、手眼标定、深度和工作空间。
- 按回车后仍没有 `map -> base_link`：在 RViz 重新使用 **2D Pose Estimate**。
- 机械臂未正常启动：检查控制模式、故障码、限位、CAN、示教模式和夹爪。

测试：

```bash
source /opt/ros/humble/setup.bash
source nav_grasp_real/src/nav_grasp/install/setup.bash
cd nav_grasp_real/src/nav_grasp
colcon test --packages-select nav_grasp_mission
colcon test-result --verbose
```

更底层的接口和参数说明见
[`src/nav_grasp_mission/README.md`](src/nav_grasp_mission/README.md)。

## 8. 作者

导航—抓取—返航状态机及 `nav_grasp_real/src/nav_grasp/` 由
[@lljj123](https://github.com/lljj123) 独立完成和维护。

贡献者列表见 [`../../../CONTRIBUTORS.md`](../../../CONTRIBUTORS.md)，协作规则见
[`../../../CONTRIBUTING.md`](../../../CONTRIBUTING.md)。
