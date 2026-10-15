# TRACER + Piper 两轮导航抓放

开发工作区：本仓库根目录。

任务依次抓取红色、绿色物块，两轮都导航至相同放置点并松爪复位，最后返回任务开始时由
`map -> base_link` TF 记录的完整位姿。第二次放置目标仍为放置点。

## 编译与运行

```bash
cd nav_grasp_sim/src/tracer_navigate_w_grab
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_PACKAGE_PATH
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to tracer_pick_place_mission
source install/setup.bash
ros2 launch tracer_pick_place_mission tracer_pick_place_system.launch.py
```

顶层 launch 管理仿真、唯一一套 robot_state_publisher 与控制器、Nav2 定位和导航、Nav2 RViz、
腕部相机 rqt 窗口、独立高亮日志终端、MoveIt、抓放 action server 和任务节点。
AMCL 使用已知仿真出生位姿初始化。
默认 `auto_start=false`，依赖在启动任务时检查。
入口清理其他工作区遗留的 ROS/Gazebo 路径，再 source Humble 和本工作区的
`install/local_setup.bash`，避免构建时记录的旧 `arm_ws` 环境链被再次加载。

无界面验证：

```bash
ros2 launch tracer_pick_place_mission tracer_pick_place_system.launch.py \
  gazebo_gui:=false rviz:=false camera_view:=false mission_console:=false
```

在另一个已 source 同一工作区的终端启动任务：

```bash
ros2 service call /pick_place_mission/start std_srvs/srv/Trigger '{}'
ros2 topic echo /pick_place_mission/state --qos-durability transient_local
```

任务 start service 立即返回是否已启动；实际结果由 state 话题报告。
`Ctrl+C` 由顶层 launch 统一关闭子进程。

## 参数

`config/mission_params.yaml` 是唯一任务点位来源。`pick_pose` 和 `place_pose`
均为 `[x,y,z,qx,qy,qz,qw]`。当前放置点为
`[-3.97,3.41,0,0,0,0.656059029,0.754709580]`，保留已实测能到达的停车点。
Python 状态机不包含点位常量。`object_colors` 必须为 `[red, green]`。

系统 launch 可覆盖 `world`、`map`、`nav2_params`、`mission_params`、`use_sim_time`、
`gazebo_gui`、`rviz`、`camera_view`、`mission_console`。相应的
`TRACER_GAZEBO_GUI`、`TRACER_START_RVIZ`、`TRACER_START_CAMERA_VIEW` 和
`TRACER_START_MISSION_CONSOLE` 环境变量也可用。
任务节点单独启动：

```bash
ros2 launch tracer_pick_place_mission tracer_pick_place_mission.launch.py
```

## 状态与停止

正常顺序：

```text
IDLE → WAIT_DEPENDENCIES → CAPTURE_START_POSE
→ NAV_PICK_1 → WAIT_STILL_PICK_1 → PICK_1
→ NAV_PLACE_1 → WAIT_STILL_PLACE_1 → PLACE_1
→ NAV_PICK_2 → WAIT_STILL_PICK_2 → PICK_2
→ NAV_PLACE_2 → WAIT_STILL_PLACE_2 → PLACE_2
→ RETURN_START → DONE
```

Reliable + Transient Local 的状态话题保留最近 20 个状态。抓放 feedback 写入日志，
不增加或修改上述任务状态。错误信息写入任务节点日志，状态严格为 `ERROR`。

每个 action 必须接受目标且最终返回 `STATUS_SUCCEEDED`；抓放还必须返回 `success=true`。
超时、拒绝或其他失败立即取消当前 action，发布一次零速度并停止后续任务。
延迟接受的目标在响应到达后也会被取消。停车要求持续收到新里程计，并连续满足速度阈值；
运动、数据间断或非有限速度都会打断静止区间。

```bash
ros2 service call /pick_place_mission/cancel std_srvs/srv/Trigger '{}'
ros2 service call /pick_place_mission/reset std_srvs/srv/Trigger '{}'
```

用户取消产生 `CANCELED`。任务或 action 尚未结束时拒绝 reset；终态 reset 后才能重新 start。
reset 只重置任务状态，不释放已经夹住的物块，也不重置 Gazebo 场景。
若错误发生在闭爪之后，需要在停车状态下单独调用 PLACE 释放，再重建场景中的两物块后重跑。

旧 `tracer_pick_mission` 仅保留在第一版工作区，2.0 不再包含该包。
新任务不调用 `/pick_mission/start` 或 `/manipulation/manipulate_object`。

## 当前物理夹持限制

保留现有视觉与机械臂抓取标定、50 mm 动态物块和 46 mm 闭爪目标。
工作空间 `workspace.max_z_m` 调整为 0.12 m：真实仿真观测到补偿后的 TCP 高度
0.103 m，比原 0.100 m 上限高 3 mm；其余工作空间与运动规划检查照常执行。
`/manipulation/has_payload` 表示抓放 server 的软件携物状态。闭爪控制器成功后设置为 true，
PLACE 松爪成功后为 false，并非接触传感器确认。
MoveIt PlanningScene 的附着不会在 Gazebo 中生成物理连接；现有摩擦参数 `mu=1.5` 下的
滑移问题仍需通过真实仿真抓取与运输观察验收。这里没有增加物块真值定位、物理吸附或
自动补偿滑移逻辑。PLACE 使用固定箱心 `map=(-4.0,4.0)`，执行时通过真实
`map -> piper_base_link` TF 计算当前机械臂目标，不依赖理想放置导航点。
规划或 TF 失败则保持夹持并返回错误。
