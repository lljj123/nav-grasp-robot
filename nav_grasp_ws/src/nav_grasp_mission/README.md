# nav_grasp_mission

这是一个只负责“任务编排”的 ROS 2 Humble 包。它以 YASMIN 为状态机基线，
调用已经调通的 Nav2 和 `visual_grasp_ws` 接口，不包含也不修改导航、感知、
抓取算法和机械臂控制代码。

任务流程如下：

```text
检查依赖 -> 记录起点 -> 导航到抓取点 -> 等待底盘静止
        -> 生成抓取计划 -> 执行完整抓取 -> 等待搬运姿态
        -> 导航回本次任务记录的起点 -> 完成
```

任意步骤失败或超时都会进入 `ERROR`，不会继续执行后续步骤；取消任务会进入
`CANCELED`。只有收到抓取执行器的 `SUCCEEDED` 后才会返航。

到达抓取点并确认底盘静止后，`PLAN_PICK` 会提供 15 秒目标检测窗口，每 0.5 秒
重新请求一次抓取规划。单帧未检测到目标或检测数据暂时过期只会继续搜索，不会
立即进入 `ERROR`；只有整个 15 秒窗口仍没有有效目标时才会安全超时退出，机械臂
不会因此开始运动。

## 边界与接口

本包只使用已有接口：

- Nav2 action：`/navigate_to_pose`
- 抓取规划：`/visual_grasp/plan_pick` (`std_srvs/Trigger`)
- 完整抓取：`/visual_grasp/execute_full_pick` (`std_srvs/Trigger`)
- 抓取状态：`/visual_grasp/execution_state` (`std_msgs/String`)
- 定位与安全检查：`map -> base_link` TF、`/odom`

状态机节点本身不会启动或关闭 `slam_ws`、`visual_grasp_ws`，也不会直接发送机械
臂关节、末端或夹爪指令；根目录的一键启动器负责这些外部组件的启停和安全收尾。
抓取颜色和机械臂参数仍由 `visual_grasp_ws` 管理。

## 安装与构建

先安装 YASMIN：

```bash
sudo apt update
sudo apt install ros-humble-yasmin ros-humble-yasmin-ros
```

然后构建本工作空间：

```bash
./nav_grasp_ws/build.sh
```

## 首次配置

复制公开的安全默认配置，再编辑本机文件：

```bash
cp nav_grasp_ws/src/nav_grasp_mission/config/mission.yaml \
   nav_grasp_ws/src/nav_grasp_mission/config/mission.local.yaml
```

在 `mission.local.yaml` 中设置：

```yaml
target_pose_configured: true
target_pose_xyyaw: [目标点地图坐标x, 目标点地图坐标y, 朝向yaw弧度]
```

`target_pose_configured` 默认是 `false`。在目标点未实测并改为 `true` 前，
`/nav_grasp/start` 会拒绝启动，因此占位坐标 `[0, 0, 0]` 不会造成误运动。

每次任务的返航点不写死：状态机会在出发前从 TF 记录机器人当时在 `map` 中的
位姿，抓取成功后导航回该位姿。

## 运行

推荐直接使用工作空间根目录的一键启动器：

```bash
./nav_grasp_ws/start_full_mission.sh \
  --map /absolute/path/to/map.yaml \
  --mission-config /absolute/path/to/mission.local.yaml
```

启动器读取地图和目标点、配置 CAN，自动打开并依次等待底盘/雷达、D435、
Nav2/RViz、机械臂（使用现有 `start_arm_control.sh --replace`）、视觉抓取和任务
状态机。所有组件就绪后，在 RViz 使用 **2D Pose Estimate** 完成定位并确认现场
安全；回到启动器终端按一次回车后，状态机才会开始导航—抓取—返航。

如果启动前检测到上一次遗留的同项目组件，启动器会先取消旧任务、停止底盘，
必要时调用机械臂软件急停，然后只清理本项目的导航、相机、视觉抓取和状态机
进程，再启动新的一套组件；不会全局终止其他 ROS 或 Python 程序。

主控终端会持续显示任务状态。任务结束后按回车，或随时在主控终端按 `Ctrl-C`，
它都会关闭本次启动的全部进程和终端。运行中退出会先取消任务、发送底盘零速度；
若检测到机械臂仍在执行抓取动作，还会先调用软件急停。实体危险仍须优先使用实体
急停。

以下是需要单独调试组件时的手动启动方式。

先按原项目方式分别启动已经调好的子系统。下面是现有工程对应的命令示例。

终端 1，启动保存地图导航（地图路径按实际情况修改）：

```bash
source $NAV_GRASP_ROOT/slam_ws/setup.zsh
ros2 launch nav navigation.launch.py \
  map:=$NAV_GRASP_ROOT/slam_ws/maps/map.yaml \
  start_joint_state_publisher:=false
```

在 RViz 中完成初始定位，并确认 Nav2 可以正常接收目标。

终端 2，启动机械臂驱动：

```bash
cd $NAV_GRASP_ROOT/visual_grasp_ws
./start_arm_control.sh --replace
```

终端 3，启动允许真实机械臂运动的视觉抓取栈：

```bash
cd $NAV_GRASP_ROOT/visual_grasp_ws
./start_perception.sh allow_robot_commands:=true
```

终端 4，启动任务状态机：

```bash
source /opt/ros/humble/setup.zsh
source $NAV_GRASP_ROOT/nav_grasp_ws/install/setup.zsh
ros2 launch nav_grasp_mission mission.launch.py
```

状态机节点启动本身不会移动机器人。确认现场安全、目标点配置正确且三个子系统
都已就绪后，再显式开始任务：

```bash
ros2 service call /nav_grasp/start std_srvs/srv/Trigger '{}'
```

## 状态、取消与复位

查看当前状态：

```bash
ros2 service call /nav_grasp/status std_srvs/srv/Trigger '{}'
ros2 topic echo /nav_grasp/state
ros2 topic echo /nav_grasp/event
```

`/nav_grasp/state` 的主要值为：

```text
IDLE
WAIT_DEPENDENCIES
CAPTURE_START_POSE
NAVIGATE_TO_TARGET
WAIT_BASE_STILL
PLAN_PICK
EXECUTE_PICK
WAIT_PICK_RESULT
RETURN_TO_START
DONE / ERROR / CANCELED
```

取消任务：

```bash
ros2 service call /nav_grasp/cancel std_srvs/srv/Trigger '{}'
```

导航阶段取消会请求 Nav2 取消当前目标，并发布一次零速度。抓取阶段取消只停止
状态机后续流程，不会中断已经由 `visual_grasp_ws` 接受的机械臂动作，也不会开始
返航；需要紧急停止机械臂时必须使用原机械臂系统的急停方式。

任务结束、报错或取消后，检查现场和子系统状态，再把任务机恢复到 `IDLE`：

```bash
ros2 service call /nav_grasp/reset std_srvs/srv/Trigger '{}'
```

该复位只清除本包的任务状态，不复位、不松开也不移动机械臂。当前流程也不包含
放置动作；抓取成功返航时夹爪保持 `visual_grasp_ws` 定义的搬运状态。

## 关键参数

公开默认参数位于 `config/mission.yaml`，实机参数应放在被 Git 忽略的
`config/mission.local.yaml`。参数包括导航超时、抓取超时、里程计新鲜度、
底盘静止速度阈值和连续静止时间。启动时也可换用另一份参数文件：

```bash
ros2 launch nav_grasp_mission mission.launch.py \
  params_file:=/absolute/path/to/mission.yaml
```

测试命令：

```bash
cd $NAV_GRASP_ROOT/nav_grasp_ws
colcon test --packages-select nav_grasp_mission
colcon test-result --verbose
```
