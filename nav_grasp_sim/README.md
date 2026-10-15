# TRACER + Piper 导航抓放工作区

这是一个面向 ROS 2 Humble 的移动操作机器人仿真工作区。系统把 TRACER 2.0 移动底盘、Piper 六轴机械臂、腕部 RGB-D 相机、Nav2、MoveIt 2 和 Gazebo Fortress 组合在一起，完成“导航到物块 → 视觉抓取 → 导航到储物箱 → 放置”的两轮任务，最后返回任务启动位置。

工作区只保留可提交的源码；`build/`、`install/`、`log/` 等 colcon 生成目录由 `.gitignore` 排除。本文档位于仿真总目录根部。

## 文件系统

```text
nav-grasp-robot/
└── nav_grasp_sim/
    ├── README.md                         # 本说明文件
    └── src/
        └── tracer_navigate_w_grab/
            ├── .gitignore
            └── src/
                ├── agx_arm_description/          # Piper 机械臂模型与网格
                ├── piper_vision_gazebo/          # Piper + D435 相机模型
                ├── realsense2_description/       # RealSense D435 描述文件
                ├── tracer_gazebo_sim/            # 整车仿真、地图、Nav2/控制器参数
                ├── tracer_piper_moveit_config/   # MoveIt 规划组、运动学与控制器配置
                ├── tracer_piper_color_grasp/     # RGB-D 颜色识别和抓放 Action Server
                └── tracer_pick_place_mission/    # 导航与机械臂任务状态机
```

各功能包的许可证以各自 `package.xml` 和包内许可证文件为准。

## 主要脚本与功能

下表路径均相对于工作区 `nav_grasp_sim/src/tracer_navigate_w_grab/`。

| 文件 | 作用 |
| --- | --- |
| `src/tracer_pick_place_mission/tracer_pick_place_mission/mission_node.py` | 核心任务状态机；调用 Nav2 的 `NavigateToPose` 以及抓取、放置 Action，并处理超时、取消和错误状态。 |
| `src/tracer_pick_place_mission/tracer_pick_place_mission/mission_console.py` | 订阅任务事件，在终端中用颜色显示导航、抓取、放置和错误日志。 |
| `src/tracer_pick_place_mission/launch/tracer_pick_place_system.launch.py` | 一键组合 Gazebo、Nav2、RViz、MoveIt、抓放服务和任务节点。 |
| `src/tracer_piper_color_grasp/src/color_pose_estimator.cpp` | 从腕部 RGB、深度图和 CameraInfo 中分割红/黄/绿色目标，计算并稳定输出三维目标点。 |
| `src/tracer_piper_color_grasp/src/color_grasp_server.cpp` | 实现 `/manipulation/pick` 和 `/manipulation/place`；使用 MoveIt 规划机械臂轨迹并控制夹爪。 |
| `src/tracer_gazebo_sim/launch/tracer_piper_sim.launch.py` | 启动整车 Gazebo 仿真、ROS-Gazebo 桥、控制器、EKF、robot_state_publisher 和可选 RViz。 |
| `src/tracer_gazebo_sim/scripts/set_piper_initial_pose.py` | 启动后下发并校验机械臂、夹爪的初始关节位置。 |
| `src/tracer_piper_moveit_config/launch/move_group.launch.py` | 加载机器人描述、SRDF、运动学、OMPL 和轨迹控制器，启动 MoveIt `move_group`。 |

导航抓放的调用关系如下：

```text
任务状态机
├── /navigate_to_pose  ──> Nav2 控制 TRACER 底盘
├── /manipulation/pick ──> RGB-D 检测 + MoveIt + 夹爪闭合
└── /manipulation/place ─> 根据实际停车 TF 对准储物箱 + 松爪复位
```

## 状态机

任务启动时先等待 Nav2、抓取服务、放置服务和 `map -> base_link` TF，然后记录当前位置。默认依次抓取红色、绿色物块：

```text
IDLE
  → WAIT_DEPENDENCIES
  → CAPTURE_START_POSE
  → NAV_PICK_1 → WAIT_STILL_PICK_1 → PICK_1
  → NAV_PLACE_1 → WAIT_STILL_PLACE_1 → PLACE_1
  → NAV_PICK_2 → WAIT_STILL_PICK_2 → PICK_2
  → NAV_PLACE_2 → WAIT_STILL_PLACE_2 → PLACE_2
  → RETURN_START
  → DONE
```

用户取消进入 `CANCELED`，依赖、导航、视觉、规划或执行失败进入 `ERROR`。每次机械臂动作前都会检查底盘是否持续静止；Action 被拒绝、超时或失败时会取消当前目标并向 `/cmd_vel` 发布一次零速度。

## 构建

运行环境为 ROS 2 Humble。首次使用先安装依赖，再构建整个工作区：

```bash
cd nav_grasp_sim/src/tracer_navigate_w_grab
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 运行

启动完整仿真系统：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tracer_pick_place_mission tracer_pick_place_system.launch.py
```

无 Gazebo GUI、RViz、相机窗口和独立日志终端运行：

```bash
ros2 launch tracer_pick_place_mission tracer_pick_place_system.launch.py \
  gazebo_gui:=false rviz:=false camera_view:=false mission_console:=false
```

系统就绪后，在另一个已经 source 工作区的终端启动任务：

```bash
ros2 service call /pick_place_mission/start std_srvs/srv/Trigger '{}'
```

查看状态和事件：

```bash
ros2 topic echo /pick_place_mission/state --qos-durability transient_local
ros2 topic echo /pick_place_mission/event --qos-durability transient_local
```

取消、复位任务：

```bash
ros2 service call /pick_place_mission/cancel std_srvs/srv/Trigger '{}'
ros2 service call /pick_place_mission/reset std_srvs/srv/Trigger '{}'
```

## 关键配置

- `src/tracer_pick_place_mission/config/mission_params.yaml`：抓取点、放置点、颜色顺序、导航/机械臂超时和停车阈值。
- `src/tracer_piper_color_grasp/config/grasp_task.yaml`：HSV 阈值、深度检测、工作空间、抓取补偿、夹爪开度、机械臂姿态和储物箱位置。
- `src/tracer_gazebo_sim/config/nav2_params.yaml`：Nav2 定位、规划、控制和行为树参数。
- `src/tracer_gazebo_sim/config/piper_controllers.yaml`：机械臂与夹爪的 ros2_control 控制器。
- `src/tracer_piper_moveit_config/config/`：MoveIt 运动学、关节限制、OMPL 和控制器映射。

当前任务点由 `mission_params.yaml` 唯一确定，格式为 `[x, y, z, qx, qy, qz, qw]`。修改任务点或抓取标定后，应先在仿真中低速验证。

## ROS 接口

| 类型 | 名称 | 说明 |
| --- | --- | --- |
| Service | `/pick_place_mission/start` | 启动两轮任务。 |
| Service | `/pick_place_mission/cancel` | 请求取消当前任务。 |
| Service | `/pick_place_mission/reset` | 终态下重置为 `IDLE`。 |
| Action | `/navigate_to_pose` | Nav2 导航。 |
| Action | `/manipulation/pick` | 按颜色执行视觉抓取。 |
| Action | `/manipulation/place` | 在储物箱上方放置并按需复位机械臂。 |
| Topic | `/pick_place_mission/state` | 任务状态。 |
| Topic | `/pick_place_mission/event` | 面向操作员的任务事件。 |
| Topic | `/manipulation/has_payload` | 抓放服务维护的软件携物状态。 |
| Topic | `/color_grasp/state` | 抓放内部阶段。 |
| Topic | `/emergency_stop` | `std_msgs/Bool` 软件急停输入。 |

`/manipulation/has_payload` 是软件状态，不是夹爪接触传感器的真实反馈；MoveIt PlanningScene 中的附着也不等同于 Gazebo 物理连接。抓取参数改变后，需要通过仿真实际观察物块是否被抬升并随车运输。
