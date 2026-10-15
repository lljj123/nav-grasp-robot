# Nav-Grasp Robot

## 1. 项目介绍

Nav-Grasp Robot 是面向 ROS 2 Humble 的移动操作机器人项目，将二维
SLAM、Nav2 导航、RGB-D 视觉抓取和导航—抓取—返航状态机集成在同一
仓库中。代码按运行环境分为 `nav_grasp_real/` 实机程序和
`nav_grasp_sim/` 仿真程序。

```text
PointCloud2 + odom -> /scan -> SLAM Toolbox -> map
                                      |
map + /scan + /odom -> AMCL + Nav2 -> /cmd_vel
                                      |
RGB + aligned depth -> cube detector -> grasp planner -> arm executor
                                      |
                         mission FSM -> return to start
```

主要文档：

| 文档 | 内容 |
| --- | --- |
| [`README.md`](README.md) | 总项目安装、运行顺序和一键启动 |
| [`nav_grasp_real/README.md`](nav_grasp_real/README.md) | 实机目录结构和工作区入口 |
| [`nav_grasp_sim/README.md`](nav_grasp_sim/README.md) | TRACER + Piper 仿真、导航和抓放 |
| [`nav_grasp_real/src/slam/README.md`](nav_grasp_real/src/slam/README.md) | SLAM 建图与 Nav2 导航 |
| [`nav_grasp_real/src/visual_grasp/README.md`](nav_grasp_real/src/visual_grasp/README.md) | 手眼标定、方块检测和实机抓取 |
| [`nav_grasp_real/src/nav_grasp/README.md`](nav_grasp_real/src/nav_grasp/README.md) | 导航—抓取—返航状态机 |

仓库目录结构：

```text
nav-grasp-robot/
├── nav_grasp_real/
│   └── src/
│       ├── slam/
│       ├── visual_grasp/
│       └── nav_grasp/
└── nav_grasp_sim/
    └── src/
        └── tracer_navigate_w_grab/
```

主要工作区：

| 路径 | 作用 |
| --- | --- |
| `nav_grasp_real/src/slam/src/lidar_bringup` | 雷达启动和 PointCloud2 转 `/scan` |
| `nav_grasp_real/src/slam/src/slam` | SLAM Toolbox 建图与地图保存 |
| `nav_grasp_real/src/slam/src/nav` | Nav2 定位、规划、控制和安全行为树 |
| `nav_grasp_real/src/visual_grasp/src/visual_grasp_*` | 标定、RGB-D 检测、抓取规划和执行 |
| `nav_grasp_real/src/nav_grasp/src/nav_grasp_mission` | YASMIN 任务状态机 |
| `nav_grasp_sim/src/tracer_navigate_w_grab` | Gazebo、Nav2、MoveIt 2 导航抓放仿真工作区 |

> 安全提示：本项目会控制真实底盘和机械臂。实机运行前必须完成
> 标定、低速验证和工作空间检查，并确保底盘与机械臂实体急停可用。

## 2. 硬件配置与运行环境

默认实机配置：

| 类别 | 默认配置 |
| --- | --- |
| 移动底盘 | AgileX Tracer，SocketCAN 接口名和波特率按本机配置 |
| 激光雷达 | Livox MID-360，点云默认为 `/livox/lidar` |
| RGB-D 相机 | Intel RealSense D435 |
| 机械臂 | PiPER-H / AGX 驱动，默认使用 `can0` |
| 末端执行器 | AGX 夹爪，抓取目标为 30 mm 红/绿/蓝方块 |
| 计算机 | Ubuntu 22.04 x86_64，具有图形桌面 |

软件环境：

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- colcon、rosdep、vcstool
- Nav2、SLAM Toolbox、YASMIN、OpenCV、RealSense ROS 2
- Tracer、Livox 和 PiPER/AGX 对应驱动

导航和建图功能只依赖标准 ROS 2 话题和 TF，替换硬件时需保持：

```text
底盘：/odom、/cmd_vel、odom -> base_footprint TF
雷达：sensor_msgs/PointCloud2、base_link -> lidar_frame TF
导航：map -> odom -> base_footprint -> base_link -> lidar_frame
```

## 3. 运行环境相关包与依赖安装

安装基础工具和状态机依赖：

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool \
  ros-humble-yasmin ros-humble-yasmin-ros
```

首次使用 rosdep 时执行；如果本机已初始化则跳过 `rosdep init`：

```bash
sudo rosdep init
rosdep update
```

拉取仓库声明的 Tracer、Livox 和 AGX 驱动依赖，再安装 ROS 包依赖：

```bash
vcs import . < dependencies.repos
rosdep install --from-paths nav_grasp_real/src/slam/src nav_grasp_real/src/visual_grasp/src nav_grasp_real/src/nav_grasp/src \
  --ignore-src --rosdistro humble -r -y
```

Livox 驱动还需按上游说明安装 Livox SDK2。D435 和 PiPER/AGX 需先按
硬件厂商说明验证单机驱动。

复制本机配置：

```bash
cp nav_grasp_real/project.env.example nav_grasp_real/project.env
```

编辑 `nav_grasp_real/project.env`，填写 ROS 2、底盘/雷达/机械臂 overlay、相机启停脚本、
CAN 初始化脚本、地图和任务参数路径。该文件已被 Git 忽略。

构建所有工作区：

```bash
./scripts/build_all.sh
```

## 4. 正常运行命令

标准顺序是：**先用 SLAM 建图并保存地图，再用 Nav2 加载该地图导航**。
不应在没有可用地图和初始定位的情况下直接运行完整任务。

### 4.1 SLAM 建图

启动底盘和雷达后，在一个独立终端启动 SLAM Toolbox：

```bash
source nav_grasp_real/src/slam/setup.bash
ros2 launch slam mapping.launch.py
```

使用另一个终端低速遥控底盘，走完环境并形成回环；再新开终端保存地图：

```bash
mkdir -p "$PWD/nav_grasp_real/src/slam/maps"
ros2 run slam save_map "$PWD/nav_grasp_real/src/slam/maps/my_map"
```

最后一个参数是自定义保存前缀，不要添加扩展名。

完整建图命令见 [`nav_grasp_real/src/slam/README.md`](nav_grasp_real/src/slam/README.md)。

### 4.2 Nav2 导航

停止 SLAM 建图节点，保持底盘、雷达和 `/scan` 正常，加载已保存的地图：

```bash
source nav_grasp_real/src/slam/setup.bash
ros2 launch nav navigation.launch.py \
  map:="$PWD/nav_grasp_real/src/slam/maps/my_map.yaml"
```

Nav2 启动后必须：

1. 在 RViz 点击 **2D Pose Estimate**。
2. 在地图上设置机器人真实位置和朝向。
3. 确认 LaserScan 与地图墙体重合。
4. 再使用 **Nav2 Goal** 发送导航目标。

未设置或错误设置初始位姿时，AMCL 无法给出可靠定位，不得开始导航。

### 4.3 视觉抓取

相机和机械臂驱动就绪后，在终端 1 以禁止实机指令模式启动：

```bash
./nav_grasp_real/src/visual_grasp/start_perception.sh
```

保持终端 1 运行，在终端 2 请求一次检测和规划：

```bash
ros2 service call /visual_grasp/plan_pick std_srvs/srv/Trigger '{}'
```

确认标定、检测、TCP 和四段抓取位姿正确后，先在终端 1 按 `Ctrl+C`
停止只读实例，才可重新启动允许实机指令的实例：

```bash
./nav_grasp_real/src/visual_grasp/start_perception.sh allow_robot_commands:=true
```

详细标定与抓取步骤见 [`nav_grasp_real/src/visual_grasp/README.md`](nav_grasp_real/src/visual_grasp/README.md)。

### 4.4 状态机

当 Nav2、相机、机械臂和视觉抓取栈都已启动时，可单独启动状态机：

```bash
source /opt/ros/humble/setup.bash
source nav_grasp_real/src/nav_grasp/install/setup.bash
ros2 launch nav_grasp_mission mission.launch.py \
  params_file:="$PWD/nav_grasp_real/src/nav_grasp/src/nav_grasp_mission/config/mission.local.yaml"
```

确认 RViz 初始位姿和现场安全后，显式启动任务：

```bash
ros2 service call /nav_grasp/start std_srvs/srv/Trigger '{}'
```

## 5. 一键运行命令

完整项目一键启动的前提是：

- SLAM 地图已保存。
- 手眼标定和夹爪检查已完成。
- 抓取停车点已实测，`target_pose_configured: true`。
- `nav_grasp_real/project.env` 中硬件启停脚本和路径正确。

确认上述配置完成后，启动底盘、雷达、D435、Nav2/RViz、机械臂、视觉抓取和状态机：

```bash
./nav_grasp_real/src/nav_grasp/start_full_mission.sh \
  --map /absolute/path/to/map.yaml \
  --mission-config /absolute/path/to/mission.local.yaml
```

启动器会等待人工在 RViz 设置 **2D Pose Estimate** 并检查安全，只有在
主控终端按回车后才会提交导航—抓取—返航任务。详见
[`nav_grasp_real/src/nav_grasp/README.md`](nav_grasp_real/src/nav_grasp/README.md)。

## 6. 验证与本机数据

```bash
colcon test --base-paths nav_grasp_real/src/visual_grasp/src nav_grasp_real/src/nav_grasp/src
colcon test-result --verbose
```

地图、手眼标定结果、夹爪行程报告、`nav_grasp_real/project.env` 和 `*.local.yaml`
都是机器人或场地专用数据，默认不提交到 GitHub。

`nav_grasp_real/src/slam/src/pointcloud_to_laserscan` 是保留原许可证的上游源码，来源与
commit 记录见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。项目自有代码使用
Apache-2.0，详见 [LICENSE](LICENSE)。

## 7. 共同创作

项目初始版本由 [@lljj123](https://github.com/lljj123) 和
[@GGbond61](https://github.com/GGbond61) 共同完成：

- `@lljj123` 负责导航、SLAM、状态机以及 `nav_grasp_real/src/slam/`、`nav_grasp_real/src/nav_grasp/`。
- `@GGbond61` 负责视觉抓取和 `nav_grasp_real/src/visual_grasp/`。

两个独立模块在总项目中完成集成，共同构成导航—抓取机器人系统。

贡献者列表见 [CONTRIBUTORS.md](CONTRIBUTORS.md)，协作、Commit 身份和 Pull Request
规则见 [CONTRIBUTING.md](CONTRIBUTING.md)。
