# 可配置二维 SLAM 与导航（ROS 2 Humble）

工作区位置：`slam_ws`。核心链路只要求底盘提供 `/odom` 和
`/cmd_vel`，雷达提供 `sensor_msgs/PointCloud2`。默认硬件 profile 是
Tracer + Livox MID-360，但功能包名和 SLAM/Nav2 配置不绑定这些型号。

## 功能包

| 功能包 | 作用 | 主要输入/输出 |
| --- | --- | --- |
| `lidar` | 可选启动默认雷达驱动，并将 PointCloud2 投影为二维扫描 | `cloud_topic` → `/scan` |
| `slam` | SLAM Toolbox 建图及地图保存 | `/scan` + TF → `/map` |
| `nav` | Nav2 在线导航或加载地图导航 | `/map`、`/scan`、`/odom` → `/cmd_vel` |
| `robot_description` | 内置简化移动操作机器人 xacro，可替换为实机模型 | `/robot_description` + 安装 TF |

上游 `pointcloud_to_laserscan` 源码也放在 `src/`，因为本机没有安装对应的 Humble
二进制包。

## 编译

```bash
./slam_ws/build.sh
source slam_ws/setup.zsh
```

## 一键启动（推荐）

```bash
cd $NAV_GRASP_ROOT/slam_ws
./scripts/start.sh
```

菜单选择模式：

- **建图**：自动配置 CAN、后台启动底盘+雷达、新终端打开 SLAM 建图（含
  RViz 建图画面）和键盘遥控（限速 `0.3 m/s` / `0.5 rad/s`）；主终端菜单
  `s` 保存地图、`t` 重开遥控、`q` 全部停止。RViz 右侧 SlamToolboxPlugin
  面板也有 **Save Map / Save As** 按钮，点击即保存地图（已自动定位到
  `maps/` 目录）。
- **导航**：后台启动底盘+雷达，从 `maps/` 选择地图后新终端打开 Nav2
  导航（含 RViz）；主终端菜单 `r` 重启导航/换地图、`q` 全部停止。

底盘+雷达日志在 `logs/base_lidar.log`。手动分终端启动的方式见下文。


## 实车建图

确认 CAN 已配置并启动（通常为 500 kbit/s）：

```bash
sudo ip link set can1 down 2>/dev/null || true
sudo ip link set can1 type can bitrate 500000
sudo ip link set can1 up
```

终端 1：启动 Tracer 底盘，提供 `/odom`、`odom → base_footprint` 和
`/cmd_vel`。`base_footprint → base_link` 由 URDF 发布。

```bash
source $NAV_GRASP_ROOT/slam_ws/setup.zsh
ros2 launch tracer_base tracer_base.launch.py \
  port_name:=can1 base_frame:=base_footprint
```

终端 2：启动 MID-360 和三维转二维。默认安装外参为
`base_link → livox_frame = (0.15, 0, 0.10)` 米。

```bash
source $NAV_GRASP_ROOT/slam_ws/setup.zsh
ros2 launch lidar converter.launch.py publish_lidar_tf:=false
```

如果实车雷达安装位置不同，需要在启动时覆盖外参，例如：

```bash
ros2 launch lidar converter.launch.py \
  lidar_x:=0.15 lidar_y:=0.0 lidar_z:=0.30 lidar_yaw:=0.0
```

终端 3：启动 SLAM Toolbox 建图。

```bash
source $NAV_GRASP_ROOT/slam_ws/setup.zsh
ros2 launch slam mapping.launch.py
```

该命令同时从 `robot_description` 加载仓库内置的简化 xacro，
发布 `base_footprint -> base_link -> lidar_frame` 以及机械臂安装基座 TF。
实机尺寸和安装位置必须通过 launch 参数校准；也可传入接受同名
xacro 参数的完整机器人模型。它不会启动 Gazebo、MoveIt 或抓取任务。

建图期间需要同时使用 Nav2 自动行驶时，终端 4 启动：

```bash
source $NAV_GRASP_ROOT/slam_ws/setup.zsh
ros2 launch nav navigation_during_mapping.launch.py
```

该 launch 默认使用 `nav2_params_during_mapping.yaml`，速度限制为
`0.3 m/s`、`0.5 rad/s`（与建图速度建议一致）。正式导航
`navigation.launch.py` 使用 `nav2_params.yaml`，限速为 `0.3 m/s`、
`0.4 rad/s`（已调整为原正式导航速度的一半）。

建议建图速度不超过 `0.3 m/s`，转速不超过 `0.5 rad/s`。

## 保存地图

```bash
source $NAV_GRASP_ROOT/slam_ws/setup.zsh
ros2 run slam save_map $NAV_GRASP_ROOT/slam_ws/maps/map
```

生成 `map.yaml`、`map.pgm` 和可继续编辑的 `map.posegraph`。

## 使用保存地图导航

保持底盘和 `lidar` 正在运行，停止建图节点，然后启动：

```bash
source $NAV_GRASP_ROOT/slam_ws/setup.zsh
ros2 launch nav navigation.launch.py \
  map:=$NAV_GRASP_ROOT/slam_ws/maps/map.yaml
```

保存地图导航也会自动加载同一 URDF。
`start_joint_state_publisher` 公开默认为 `false`；只有在自定义模型包含可动关节
且没有实机关节状态源时才应开启。

在 RViz 中先用 **2D Pose Estimate** 设置初始位置，再用 **Nav2 Goal** 下发目标点。

## 检查

```bash
cd $NAV_GRASP_ROOT/slam_ws
./check_stack.sh
```

正常 TF 链应为：

```text
map → odom → base_footprint → base_link → livox_frame
```

三维转二维高度范围位于
`src/lidar/config/converter.yaml`，默认按 `base_link` 坐标过滤
`-0.05～0.50 m`。若 `/scan` 中地面点过多或障碍物缺失，应结合实际雷达安装高度调整。
