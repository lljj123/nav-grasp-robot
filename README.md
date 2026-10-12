# Nav-Grasp Robot

面向 ROS 2 Humble 的移动操作机器人导航、视觉抓取与任务编排工程。项目把硬件相关
内容收敛为可替换配置，SLAM 工作区功能包使用通用功能名称，不再用 MID-360、Tracer、PiPER 等
具体型号命名。

默认实机配置仍以 Tracer 底盘、Livox MID-360、Intel RealSense D435 和 PiPER-H
为参考，但导航与建图模块只依赖标准 ROS 话题、TF 和 action 接口。

> 安全提示：本项目会控制真实底盘和机械臂。公开配置默认禁止机械臂指令、禁用
> 未回零夹爪豁免、未配置任务目标。实机运行前必须完成本机配置、低速验证并准备
> 实体急停。抓取执行使用受保护的 MoveP/MoveL/MoveJ，不是 MoveIt 全场景碰撞规划。

## 系统流程

```text
PointCloud2 + odom ──> 2D scan / SLAM Toolbox / Nav2 ──┐
                                                       ├─> mission FSM ─> return
RGB + aligned depth ─> cube detector ─> grasp planner ─┘
                                           │
                                           └─> guarded arm executor
```

## 仓库结构

| 路径 | 作用 |
| --- | --- |
| `slam_ws/src/lidar` | 可配置点云转二维激光；内置 Livox 默认驱动配置 |
| `slam_ws/src/slam` | SLAM Toolbox 建图和地图保存 |
| `slam_ws/src/nav` | Nav2 定位、规划、控制及安全行为树 |
| `slam_ws/src/robot_description` | 简化通用机器人模型及底盘/雷达/机械臂安装 TF |
| `visual_grasp_ws/src/visual_grasp_*` | 手眼标定、RGB-D 检测、候选规划和抓取执行 |
| `nav_grasp_ws/src/nav_grasp_mission` | YASMIN 导航—抓取—返航状态机 |

`slam_ws/src/pointcloud_to_laserscan` 是保留原许可证的上游 vendored 源码，来源与
commit 记录在 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。

## 环境与依赖

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10
- Nav2、SLAM Toolbox、YASMIN
- 对应底盘、雷达、RGB-D 相机和机械臂的 ROS 2 驱动

获取已知的公开驱动依赖：

```bash
sudo apt install python3-vcstool python3-rosdep
vcs import . < dependencies.repos
rosdep install --from-paths slam_ws/src visual_grasp_ws/src nav_grasp_ws/src \
  --ignore-src --rosdistro humble -r -y
```

Livox 驱动还需遵循其上游说明安装 Livox SDK2。相机启动/停止脚本属于现场硬件
生命周期配置，不由本仓库猜测；通过下面的本地环境文件接入。

## 本机配置

```bash
cp config/project.env.example config/project.env
```

编辑 `config/project.env`，填写驱动 overlay、机械臂控制入口、相机启停脚本、CAN
初始化脚本和地图路径。该文件已被 Git 忽略。

任务目标同样不提交实机坐标：

```bash
cp nav_grasp_ws/src/nav_grasp_mission/config/mission.yaml \
   nav_grasp_ws/src/nav_grasp_mission/config/mission.local.yaml
```

测量本机 `map -> base_link` 下的抓取停车点后，编辑 `mission.local.yaml`，将
`target_pose_configured` 改为 `true`，再在 `config/project.env` 中设置
`NAV_GRASP_MISSION_CONFIG`。

地图、手眼结果、夹爪行程报告和 `*.local.yaml` 均按机器人或场地隔离，不进入 Git。

## 构建

```bash
./scripts/build_all.sh
```

也可分别构建：

```bash
./slam_ws/build.sh
./visual_grasp_ws/build.sh
./nav_grasp_ws/build.sh
```

## 导航与建图

```bash
source slam_ws/setup.bash
ros2 launch lidar converter.launch.py
ros2 launch slam mapping.launch.py
```

使用已有地图：

```bash
source slam_ws/setup.bash
ros2 launch nav navigation.launch.py \
  map:=/absolute/path/to/map.yaml
```

替换雷达时，可以由外部启动任意 `sensor_msgs/PointCloud2` 驱动，并使用：

```bash
ros2 launch lidar converter.launch.py \
  start_lidar:=false cloud_topic:=/my_lidar/points \
  lidar_frame:=my_lidar_frame
```

通用描述包自带一个简化 xacro。真实安装外参或完整模型可通过
`robot_description/launch/robot_description.launch.py` 的参数覆盖。

## 视觉抓取

先以只读模式检查相机、TF、检测和候选：

```bash
./visual_grasp_ws/start_perception.sh
ros2 service call /visual_grasp/plan_pick std_srvs/srv/Trigger '{}'
```

只有完成标定、工作空间检查和低速验证后，才显式允许实机命令：

```bash
./visual_grasp_ws/start_perception.sh allow_robot_commands:=true
```

详细标定和执行步骤见 [visual_grasp_ws/README.md](visual_grasp_ws/README.md)。

## 完整任务

静态检查不会启动设备：

```bash
./nav_grasp_ws/start_full_mission.sh --check \
  --map /absolute/path/to/map.yaml \
  --mission-config /absolute/path/to/mission.local.yaml
```

检查通过后去掉 `--check`，启动器仍会等待人工完成定位和安全确认，之后才提交
导航—抓取—返航任务。

## 验证

```bash
colcon test --base-paths visual_grasp_ws/src nav_grasp_ws/src
colcon test-result --verbose
```

硬件接口、话题和安全状态检查见各工作区 README。真实机器人参数不能直接照搬默认
值，尤其是雷达/机械臂安装 TF、TCP、工作空间、夹爪限制和目标地图坐标。

## 协作与分工

项目初始版本由 [@lljj123](https://github.com/lljj123) 和
[@GGbond61](https://github.com/GGbond61) 共同完成：`@lljj123` 主要维护导航、SLAM
和 `slam_ws/`，`@GGbond61` 主要维护视觉抓取和 `visual_grasp_ws/`；
`nav_grasp_ws/` 和实机联调由双方共同负责。贡献者列表见
[CONTRIBUTORS.md](CONTRIBUTORS.md)。

协作时从 `main` 创建功能分支，通过 Pull Request 合并；跨模块修改由
另一位负责人审查。分支命名、测试要求、Commit 身份和实机安全规则见
[CONTRIBUTING.md](CONTRIBUTING.md)。

## License

项目自有代码使用 Apache-2.0，详见 [LICENSE](LICENSE)。第三方代码和依赖见
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
