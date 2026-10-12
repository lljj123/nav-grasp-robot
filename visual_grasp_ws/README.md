# Visual Grasp：PiPER + D435 传统视觉抓取

面向 Ubuntu 22.04 / ROS 2 Humble 的眼在手上（eye-in-hand）抓取工程。机械臂安装
在移动底盘上，相机为 Intel RealSense D435，目标为边长 30 mm 的红、绿、蓝色
木质方块。

当前阶段包含：

- `TAG36H11-3` 单标签位姿检测与眼在手上标定；
- 手眼结果静态 TF 发布；
- D435 彩色图与对齐深度融合的三色方块检测；
- 抓取候选和预抓取/抓取/抬升/回正四段位姿规划；
- 抓取成功后从安全抬升点移动到可配置的携带姿态；
- 带反馈、超时和工作空间检查的 PiPER 两阶段执行器；
- PiPER TCP 名义变换：`piper_link6 -> piper_tcp_link`，沿法兰 Z 轴 0.142 m。

> 安全约束：启动时默认 `allow_robot_commands:=false`，不会向机械臂发送指令。
> 实机模式支持人工确认的两阶段服务，也支持显式调用一键完整抓取服务。一键模式
> 仍会先到预抓取点并稳定停留，再垂直下降。执行器使用驱动自身的 MoveP/MoveL，
> 并不等同于 MoveIt 2 全场景碰撞规划；首次运行必须低速、清空周围障碍物并随时
> 准备急停。

## 坐标系

本机既有移动机械臂模型使用带 `piper_` 前缀的坐标系，避免与小车 `base_link`
冲突：

```text
map -> odom -> base_link -> ... -> piper_base_link
                                 -> piper_link1 ... piper_link6
                                                       |-> piper_tcp_link
                                                       `-> camera_link
                                                            `-> camera_color_optical_frame
```

手眼标定直接求解 `piper_link6 -> camera_color_optical_frame`，再利用 D435 自身 TF
换算并保存 `piper_link6 -> camera_link`。运行时只发布后一条，D435 驱动继续负责
发布相机内部 TF。

## 标定板

```yaml
family: tag36h11
id: 3
tag_size_m: 0.035
```

`tag_size_m` 指 AprilTag 检测器返回的四角之间的实际边长。对当前这张复合标定板，
检测角点位于 35 mm 的内部 tag36h11 编码黑框；53 mm 外框不属于 AprilTag 编码，
不能传给 PnP。若使用不同打印版，应在调试图中核对检测角点后重新实测该尺寸。

## 构建

```bash
cd $NAV_GRASP_ROOT/visual_grasp_ws
./build.sh
source install/setup.bash
```

工程本身不复制相机或机械臂驱动。D435 继续由
`$CAMERA_START_SCRIPT` 启动；机械臂反馈/控制使用本机现有
PiPER 工作空间。标定期间必须有实时法兰位姿，且至少能查询
`piper_base_link -> piper_link6`。

机械臂控制统一从本工程启动，脚本会启用 `can0`、加载 `pyAgxArm`，并在收到
正常反馈后自动使能六个关节。运行前要支撑好机械臂、释放急停并清空工作空间：

```bash
cd $NAV_GRASP_ROOT/visual_grasp_ws
./start_arm_control.sh
```

控制节点现在直接以前台进程运行，按一次 `Ctrl-C` 会直接结束 ROS 驱动，不再
留下占用 `can0` 的后台子进程。如果升级前的旧脚本已经留下孤儿进程，确认机械臂
静止后可用下面的显式替换命令清理并重新启动：

```bash
./start_arm_control.sh --replace
```

测量夹爪编码器的手动闭合/张开端点时，先停止机械臂控制驱动，再运行：

```bash
./measure_gripper_range.sh
```

该工具先禁用夹爪，再分别等待手动移动到完全闭合和完全张开；它不发送运动目标、
不修改夹爪零点，结果默认保存到
`~/.config/nav_grasp/gripper/manual_gripper_range.json`。

## 1. 眼在手上标定

最安全的首次联调方式是启动只读 PiPER 反馈节点，它不会发送 CAN 控制帧：

```bash
cd configured arm feedback workspace
./start_arm_feedback.sh
```

然后用一条命令启动标定节点和交互采样器。该命令会把
`/arm/feedback/end_pose` 自动桥接为 `piper_base_link -> piper_link6`：

```bash
cd $NAV_GRASP_ROOT/visual_grasp_ws
./start_handeye_collection.sh
```

在另一个终端验证：

```bash
ros2 run tf2_ros tf2_echo piper_base_link piper_link6
```

如果使用 `$ARM_CONTROL_SCRIPT` 启动官方控制驱动，
改用：

```bash
./start_handeye_collection.sh arm_pose_topic:=/arm/feedback/tcp_pose
```

此时 AGX 驱动的 `tcp_offset` 必须保持全零，因为该反馈会被当作法兰位姿；
142 mm 偏移只在本工程的 `piper_link6 -> tcp_link` 中应用一次。

让标定板固定不动。如果标定节点已由 `./start_handeye.sh` 单独启动，则另开一个
终端启动交互采样器：

```bash
cd $NAV_GRASP_ROOT/visual_grasp_ws
./collect_handeye.sh
```

每次先移动机械臂，确认机械臂已经停止且标签完整可见，再在采样终端按一次回车。
节点会检查最近 1 秒位姿：平移变化不超过 1 mm、旋转变化不超过 0.5 度时，才会
保存一组“法兰位姿 + 标签位姿”；运动中误按回车不会写入样本。建议采集 20--30 个
姿态，同时改变腕部绕 X/Y/Z 的方向，避免只做平移。

交互终端中可使用：

```text
回车    采集一组
s       求解并保存结果
r       清空全部样本（需要输入 yes 确认）
h       查看帮助
q       退出采样器
```

也可以保留使用 ROS 2 服务的方式；它同样会执行稳定性检查：

```bash
ros2 service call /handeye/capture_sample std_srvs/srv/Trigger '{}'
ros2 service call /handeye/solve std_srvs/srv/Trigger '{}'
```

样本保存在 `calibration/handeye_samples.yaml`，结果保存在
`calibration/handeye.yaml`。求解器会比较 Tsai、Park、Horaud、Andreff 和
Daniilidis 方法，并按固定标定板在 `piper_base_link` 下的闭环离散程度选解。

如需清空样本：

```bash
ros2 service call /handeye/reset std_srvs/srv/Trigger '{}'
```

## 2. 方块检测与抓取候选

完成手眼标定并保持相机运行：

```bash
cd $NAV_GRASP_ROOT/visual_grasp_ws
./start_perception.sh
```

上式默认接官方控制驱动的 `/arm/feedback/tcp_pose`。如果仍使用只读反馈节点：

```bash
./start_perception.sh arm_pose_topic:=/arm/feedback/end_pose
```

主要输出：

| 名称 | 类型 | 说明 |
| --- | --- | --- |
| `/visual_grasp/blocks` | `visual_grasp_interfaces/BlockDetectionArray` | 方块中心在 `piper_base_link` 下的位姿 |
| `/visual_grasp/debug_image` | `sensor_msgs/Image` | HSV 分割和深度检查图 |
| `/visual_grasp/block_markers` | `visualization_msgs/MarkerArray` | 3D 检测标记 |
| `/visual_grasp/pick_sequence` | `geometry_msgs/PoseArray` | 预抓取、抓取、抬升、空中回正位姿 |
| `/visual_grasp/execution_state` | `std_msgs/String` | 受保护执行器当前状态 |
| `/visual_grasp/commanded_flange_pose` | `geometry_msgs/PoseStamped` | TCP补偿后的法兰目标，仅用于检查 |

### RViz2 可视化

相机、机械臂控制和感知栈启动后，另开一个终端运行：

```bash
cd $NAV_GRASP_ROOT/visual_grasp_ws
./start_rviz.sh
```

RViz2 默认以 `piper_base_link` 为固定坐标系，显示实时 PiPER-H/夹爪模型、D435
彩色点云、TCP 坐标轴、三色木块、四阶段抓取位姿以及检测调试图。显示模型使用
独立的 `model_` TF 前缀，不会覆盖抓取与手眼标定使用的实机 TF；该入口只读取
反馈，不会发送机械臂运动命令。若点云较密导致画面卡顿，可在 Displays 中临时
关闭 `D435 colored cloud`。

请求生成一次候选：

```bash
ros2 service call /visual_grasp/plan_pick std_srvs/srv/Trigger '{}'
```

默认按置信度选择任意颜色。可在
`src/visual_grasp_motion/config/grasp_planner.yaml` 中设置 `target_color` 为
`red`、`green` 或 `blue`。

规划器默认设置 `height_source: depth_center`：D435 在木块彩色轮廓内取对齐深度
中值，变换到 `piper_base_link` 后从顶面下移15 mm，得到每次检测的木块中心 Z。
因此更换箱子或工作台高度后不会继续使用标定时箱子的高度。手眼标定只确定
“相机到法兰”的刚性变换，与标定板当时放在多高无关；机械臂或相机的安装关系改变
时才需要重新做手眼标定。

旧平面参数仍保留为可选回退。只有恢复到同一个固定物理平面时，才把
`height_source` 改为 `fixed_plane`。TCP仍固定为142 mm；少量统一的高度偏差使用
`grasp_center_z_offset_m` 修正，不要修改TCP长度。

## 3. 两阶段实机抓取

### 3.1 启动前检查

官方 AGX 驱动必须使用 `agx_gripper`，并保持 `tcp_offset` 全零。本工程执行器会把
目标 TCP 按142 mm偏移反算成法兰目标；如果驱动也配置142 mm，就会发生双重补偿。

```bash
source /opt/ros/humble/setup.bash
source $ARM_DRIVER_SETUP
ros2 param get /arm/agx_arm_ctrl_single_node tcp_offset
ros2 param get /arm/agx_arm_ctrl_single_node speed_percent
ros2 param get /arm/agx_arm_ctrl_single_node effector_type
```

期望分别为全零、首次测试不高于10、`agx_gripper`。先以禁止控制模式启动并检查
识别及候选：

```bash
cd $NAV_GRASP_ROOT/visual_grasp_ws
./start_perception.sh
```

```bash
ros2 service call /visual_grasp/plan_pick std_srvs/srv/Trigger '{}'
ros2 topic echo /visual_grasp/pick_sequence --once
```

确认预抓取、抓取、抬升、回正四个 TCP 位姿后停止该 launch，再显式开启实机发布：

```bash
cd $NAV_GRASP_ROOT/visual_grasp_ws
./start_perception.sh allow_robot_commands:=true
```

### 3.2 阶段一：只到木块上方

保持机械臂和木块静止、木块完整位于相机视野内。先生成新候选，并在5秒内请求
阶段一：

```bash
ros2 service call /visual_grasp/plan_pick std_srvs/srv/Trigger '{}'
ros2 service call /visual_grasp/move_to_pregrasp std_srvs/srv/Trigger '{}'
```

当前实机测得目标需要沿 `piper_base_link` 的前方修正约 20 mm，因此规划参数
`target_offset_x_m` 设为 `0.020`。该修正同时作用于预抓取、抓取、抬升和回正位姿，
保证第二阶段仍沿同一条竖直线下降，而不是从预抓取点斜向移动。

检测器把木块图像边方向通过手眼旋转矩阵投影到 `piper_base_link`。规划器利用方块
每90度等价的特性，选择绝对值不超过约45度的最小腕部调整量，使夹爪轴与最近的
木块侧面平行。`grasp_yaw_offset_deg` 用于补偿夹爪安装的固定角度偏差，默认0度。

执行器先把夹爪张开到50 mm，再用 MoveP 到木块中心正上方90 mm。它会停在
`AWAITING_CONFIRMATION`，不会自行下降。查看状态：

```bash
ros2 service call /visual_grasp/pick_status std_srvs/srv/Trigger '{}'
```

夹爪确认抓住30 mm木块后，执行器会先沿原抓取轴垂直抬升，在空中回正夹爪。
公开配置中 `move_to_carry_after_pick: false`，不会再发送一组机器人特定的关节姿态。
若在本机 profile 中显式开启，才会通过 MoveJ 移动到
`piper_executor.yaml` 中的 `carry_joint_positions_rad`。原始实机携带
姿态来自实机现场稳定采样，并四舍五入为
`[0.00, 0.46, -0.74, 0.00, 1.05, 0.00] rad`；到位容差为6度，所以不要求每个
关节与采样值完全一致。该过程只保持木块，不会自动松开夹爪。

### 3.3 阶段二：人工确认后下降抓取

现场确认 TCP 确实位于木块正上方、夹爪方向正确、下降路径无障碍后，才执行：

```bash
ros2 service call /visual_grasp/execute_pick std_srvs/srv/Trigger '{}'
```

如果预抓取位置已经验证正确，也可以把阶段一和阶段二合并为一次调用。先生成最新
规划，然后立即请求完整抓取：

```bash
ros2 service call /visual_grasp/plan_pick std_srvs/srv/Trigger '{}'
ros2 service call /visual_grasp/execute_full_pick std_srvs/srv/Trigger '{}'
```

完整抓取仍按“张开夹爪、按木块角度移动到上方、稳定并停留1秒、保持角度垂直
下降、闭合验证、保持角度垂直抬升、在安全高度回正”依次执行，不会在靠近工作台
时边抬边转。

也可以直接使用工程入口完成上述两次服务调用：

```bash
cd $NAV_GRASP_ROOT/visual_grasp_ws
./pick_once.sh
```

手动拖动PiPER会让固件从CAN控制模式（`ctrl_mode=1`）切换到示教模式
（`ctrl_mode=2`）。`pick_once.sh` 会先调用 `/arm/enter_can_control`；该服务只恢复
控制模式，不发送位姿目标。如果仍在拖动或执行示教轨迹，服务会拒绝接管。单独恢复
时可运行：

```bash
ros2 service call /arm/enter_can_control std_srvs/srv/Trigger '{}'
```

该阶段依次执行 MoveL下降、1 N低速闭合夹爪、按夹爪宽度确认木块、MoveL抬升
100 mm，再用MoveP保持TCP位置回到正角度。夹爪固件没有独立速度字段，因此执行器
按每0.2秒收紧4 mm的方式分段闭合，平均目标变化约20 mm/s。
夹爪在闭合、抬升和回正后检查宽度；20--40 mm表示夹住约30 mm木块，接近完全
闭合或仍然过宽均为失败。一键抓取失败时会先垂直抬升、张开并等待视觉稳定，重新
检测和规划后自动再抓一次；第二次仍失败才进入 `ERROR`。人工两阶段模式的第一次
下降仍需人工确认；调用阶段二后，也允许上述一次自动重试。完成或排除故障后重置：

```bash
ros2 service call /visual_grasp/reset_pick std_srvs/srv/Trigger '{}'
```

执行中如出现异常，使用机械臂实体急停；软件急停命令为：

```bash
ros2 service call /arm/emergency_stop std_srvs/srv/Empty '{}'
```

### 3.4 `OPENING timed out` 或 `homing_status=false`

这表示夹爪还没有可靠的机械零位。先停止视觉抓取 launch，再在官方AGX驱动终端
按 `Ctrl-C`，确认两指之间没有物体，然后运行：

```bash
cd $NAV_GRASP_ROOT/visual_grasp_ws
./calibrate_gripper.sh
```

脚本会先禁用夹爪；用手轻轻把两指完全合拢后，按提示输入大写 `CALIBRATE`。
它将当前闭合位置设置为零点、配置70 mm最大行程，并自动验证能否张开到10 mm。
校准成功后重新启动AGX驱动，再启动抓取节点。不要在夹爪未完全闭合或中间有物体时
确认回零，否则后续宽度反馈和抓取力都会不正确。

当前这台夹爪在固件 `S-V1.9-1` 下能够实际执行位置命令，但始终反馈
`homing_status=false`。已使用 `./measure_gripper_range.sh` 独立测得闭合端
`-0.00007 m`、张开端 `0.07973 m`、稳定行程 `0.07980 m`。执行器不会伪造
`homing_status`；本机 profile 还必须显式设置
`allow_verified_unhomed_gripper: true`，然后执行器才会验证
`~/.config/nav_grasp/gripper/manual_gripper_range.json` 的固件、方向、端点、
行程和稳定度后才允许降级运行，并把所有夹爪命令硬限制在 `0--70 mm`。报告丢失、
内容不符或实时宽度越界时仍会拒绝抓取。

## 验收顺序

1. RViz 检查 `piper_link6 -> tcp_link` 的方向和 142 mm 长度。
2. 标定时确认 AprilTag 坐标轴稳定、重投影误差通常小于 1 px。
3. 使用未参与求解的机械臂姿态验证固定标签在 `piper_base_link` 下不漂移。
4. 用尺测量多个已知点，确认方块中心坐标误差。
5. 禁止控制模式下查看四段位姿，确认 TCP 朝向、工作台高度和机械臂可达范围。
6. 低速执行阶段一，只验证夹爪是否到达木块正上方。
7. 人工确认后执行阶段二，并检查夹爪反馈宽度和抬升方向。

## 已知待确认项

- 旧机械臂工作空间曾按固件信息配置为 `piper_h`。若实机铭牌是 PiPER-H，后续
  MoveIt 模型也必须选择 `piper_h`，不能仅按通用名称 PiPER 配置。
- `0.142 m` 当前是名义 TCP；正式抓取前仍应做定点旋转法复核 X/Y/Z 偏移。
- RGB 阈值是首版通用值，需要在实际工作台光照下通过 debug 图微调。
