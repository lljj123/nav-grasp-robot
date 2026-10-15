# SLAM 建图与 Nav2 导航

## 1. 项目介绍

`nav_grasp_real/src/slam` 是移动底盘的二维建图与导航工作区，使用 SLAM Toolbox
生成地图，使用 AMCL + Nav2 在已保存地图上定位、规划和控制底盘。

标准使用顺序是：

```text
1. SLAM 建图 -> 2. 保存 map.yaml/map.pgm -> 3. 停止建图
       -> 4. Nav2 加载地图 -> 5. RViz 设置初始位姿 -> 6. 发送导航目标
```

**必须先完成 SLAM 建图，再进行 Nav2 导航。Nav2 启动后必须在 RViz 中
设置初始位姿，否则 AMCL 没有可靠的机器人位置。**

数据流：

```text
PointCloud2 -> lidar_bringup -> pointcloud_to_laserscan -> /scan
/scan + /odom + TF -> SLAM Toolbox -> /map + map -> odom
map.yaml + /scan + /odom -> AMCL + Nav2 -> /cmd_vel -> 底盘
```

主要功能包：

| 功能包 | 作用 |
| --- | --- |
| `lidar_bringup` | 启动可配置雷达驱动，调用 `pointcloud_to_laserscan` 生成 `/scan` |
| `slam` | 启动 SLAM Toolbox、RViz 并保存地图 |
| `nav` | 启动 map server、AMCL、Nav2、RViz 和安全行为树 |
| `robot_description` | 发布底盘、雷达和机械臂安装 TF |

## 2. 硬件配置与运行环境

默认实机配置：

| 类别 | 默认配置 |
| --- | --- |
| 底盘 | AgileX Tracer |
| 底盘总线 | SocketCAN，接口名和波特率按本机实际设备配置 |
| 雷达 | Livox MID-360 |
| 默认点云 | `/livox/lidar`，`sensor_msgs/PointCloud2` |
| 建图扫描 | `/scan`，`sensor_msgs/LaserScan` |
| 底盘接口 | `/odom`、`/cmd_vel`、`odom -> base_footprint` |

软件环境：

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- Nav2、SLAM Toolbox、RViz2
- Tracer ROS 2 驱动、Livox ROS Driver 2 和 Livox SDK2

替换底盘或雷达时，只要保持下列标准接口即可：

```text
/odom
/cmd_vel
/scan
map -> odom -> base_footprint -> base_link -> lidar_frame
```

## 3. 运行环境相关包与依赖安装

以下命令默认在仓库根目录 `nav-grasp-robot/` 执行。

安装基础工具、SLAM Toolbox、Nav2 和键盘遥控包：

```bash
sudo apt update
sudo apt install can-utils python3-colcon-common-extensions python3-rosdep python3-vcstool \
  ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-teleop-twist-keyboard
```

拉取默认的 Tracer 和 Livox 驱动，再使用 rosdep 安装工作区依赖：

```bash
vcs import . < dependencies.repos
rosdep install --from-paths nav_grasp_real/src/slam/src \
  --ignore-src --rosdistro humble -r -y
```

Livox ROS Driver 2 还需按上游说明安装 Livox SDK2。

配置本机环境：

```bash
cp nav_grasp_real/project.env.example nav_grasp_real/project.env
```

至少检查 `ROS_SETUP`、`BASE_DRIVER_SETUP`、`LIDAR_DRIVER_SETUP`、
`BASE_CAN_INTERFACE`、`BASE_CAN_BITRATE`、`LIDAR_CLOUD_TOPIC` 和 `LIDAR_FRAME`。

编译：

```bash
./nav_grasp_real/src/slam/build.sh
```

每个新 Bash 终端加载：

```bash
source nav_grasp_real/src/slam/setup.bash
```

Zsh 终端改用：

```zsh
source nav_grasp_real/src/slam/setup.zsh
```

## 4. 正常运行命令

### 4.1 SLAM 建图

建图前确认底盘和雷达上电、实体急停可用、周围无人和易碰撞物。

底盘 CAN 接口名由 USB-CAN 设备、系统规则和用户配置决定，不一定是
`can0` 或 `can1`。先列出本机网络接口，找到实际使用的 SocketCAN 接口：

```bash
ip -brief link
```

将下面的 `YOUR_CAN_INTERFACE` 替换成实际接口名，确认接口为 `UP`，并短暂
观察是否能收到 CAN 帧：

```bash
CAN_IFACE=YOUR_CAN_INTERFACE
ip -details -statistics link show "$CAN_IFACE"
timeout 5 candump "$CAN_IFACE"
```

如果接口不存在、不是 `UP`、波特率不正确或完全没有数据，应先按照所用
USB-CAN 设备和底盘说明完成配置。确认通信后，把实际接口名和波特率写入
`nav_grasp_real/project.env`：

```bash
BASE_CAN_INTERFACE=YOUR_CAN_INTERFACE
BASE_CAN_BITRATE=YOUR_CAN_BITRATE
```

终端 1，启动底盘，提供 `/odom`、`/cmd_vel` 和 `odom -> base_footprint`：

```bash
source nav_grasp_real/src/slam/setup.bash
ros2 launch tracer_base tracer_base.launch.py \
  port_name:="$BASE_CAN_INTERFACE" base_frame:=base_footprint
```

终端 2，如果本项目负责启动 Livox 雷达，同时把点云转换成 `/scan`：

```bash
source nav_grasp_real/src/slam/setup.bash
ros2 launch lidar_bringup converter.launch.py publish_lidar_tf:=false
```

`publish_lidar_tf:=false` 用于避免与 `robot_description` 重复发布
`base_link -> lidar_frame`。

如果已经在另一个终端启动了雷达驱动，并且该驱动正在发布
`sensor_msgs/PointCloud2` 点云，则不应重复启动第二个雷达驱动。此时只启动
点云到 `/scan` 的转换，并把点云话题和坐标系替换成实际值：

```bash
ros2 launch lidar_bringup converter.launch.py \
  start_lidar:=false publish_lidar_tf:=false \
  cloud_topic:=/my_lidar/points lidar_frame:=my_lidar_frame
```

终端 3，启动 SLAM Toolbox、机器人模型和 RViz 建图界面：

```bash
source nav_grasp_real/src/slam/setup.bash
ros2 launch slam mapping.launch.py
```

终端 4，启动键盘遥控：

```bash
source nav_grasp_real/src/slam/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel
```

建图时建议线速度不超过 `0.3 m/s`，角速度不超过 `0.5 rad/s`。
先沿场地外围缓慢行驶，再补充内部通道，最后回到起点附近形成回环。

保存地图：

```bash
mkdir -p "$PWD/nav_grasp_real/src/slam/maps"
ros2 run slam save_map "$PWD/nav_grasp_real/src/slam/maps/my_map"
```

`save_map` 后面的引号内可以填写任意保存路径和文件名，不需要添加扩展名。
保存后停止 `mapping.launch.py`，不要让 SLAM Toolbox 和 AMCL 同时发布
`map -> odom`。

### 4.2 Nav2 导航

导航前必须已完成上述 SLAM 建图和地图保存，同时保持底盘、雷达和
`/scan` 运行。

启动 map server、AMCL、Nav2 和 RViz：

```bash
source nav_grasp_real/src/slam/setup.bash
ros2 launch nav navigation.launch.py \
  map:="$PWD/nav_grasp_real/src/slam/maps/my_map.yaml"
```

将 `my_map.yaml` 替换为刚才实际保存的地图文件；使用其他目录时也可以传入
绝对路径。

导航节点就绪后，**必须设置初始位姿**：

1. 在 RViz 中点击 **2D Pose Estimate**。
2. 在地图上点击机器人当前真实位置。
3. 拖动箭头指向机器人当前真实朝向。
4. 确认 LaserScan 与地图墙体基本重合。
5. 偏差明显时重新设置，不要发送导航目标。

完成初始定位后，再在 RViz 中使用 **Nav2 Goal** 设置目标位置和朝向。
首次测试应选择近距离、开阔区域，并准备好实体急停。

## 5. 一键运行命令

在图形桌面终端中运行：

```bash
./nav_grasp_real/src/slam/scripts/start.sh
```

首次使用顺序：

1. 选择“建图”。
2. 等待 `/scan`、SLAM/RViz 和键盘遥控终端就绪。
3. 低速走完环境，回到主控终端按 `s`，输入自定义保存路径前缀。
4. 按 `q` 停止建图及硬件进程。
5. 重新运行脚本，选择“导航”并选择刚保存的地图。
6. Nav2/RViz 启动后，使用 **2D Pose Estimate** 设置初始位姿。
7. 确认激光与地图重合后，再使用 **Nav2 Goal** 导航。

建图和导航不是一次菜单内连续跳过的步骤；应先生成可用地图，再单独进入导航模式。

## 6. 常见问题

### 没有 `/scan`

- 检查雷达驱动是否发布 PointCloud2。
- 检查 `cloud_topic` 和点云 `frame_id`。
- 检查 `src/lidar_bringup/config/converter.yaml` 的高度与距离过滤。

### 没有 `/map` 或地图重影

- 确认 `/scan`、`/odom` 和 TF 持续更新。
- 降低底盘速度，检查里程计方向和尺度。
- 检查雷达安装外参和时间戳。

### 导航定位跳变

- 重新使用 **2D Pose Estimate** 设置正确初始位姿。
- 检查是否同时启动 SLAM Toolbox 和 AMCL；二者不应同时发布
  `map -> odom`。
- 检查 `base_link -> lidar_frame` 是否被两个节点重复发布。

### 导航有路径但底盘不动

- 检查 `/cmd_vel`、CAN、底盘上电、急停和驱动安全锁。
- 确认底盘驱动订阅的速度话题是 `/cmd_vel`。

主要参数：

| 文件 | 用途 |
| --- | --- |
| `src/lidar_bringup/config/converter.yaml` | PointCloud2 转 LaserScan 过滤参数 |
| `src/slam/config/slam_toolbox.yaml` | 建图、扫描匹配和回环参数 |
| `src/nav/config/nav2_params.yaml` | AMCL、规划、控制和成本地图参数 |
| `src/nav/behavior_trees/` | Nav2 安全行为树 |
| `src/robot_description/urdf/` | 机器人模型和安装 TF |

## 7. 作者

SLAM、导航及 `nav_grasp_real/src/slam/` 由 [@lljj123](https://github.com/lljj123)
独立完成和维护。

贡献者列表见 [`../../../CONTRIBUTORS.md`](../../../CONTRIBUTORS.md)，协作规则见
[`../../../CONTRIBUTING.md`](../../../CONTRIBUTING.md)。
