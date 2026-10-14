# TRACER Piper 颜色视觉抓放

本包通过腕部 RGB-D 相机识别指定颜色，并用 MoveIt 与夹爪控制器执行抓放。
外部接口是两个独立 action：

- `/manipulation/pick`：`tracer_piper_color_grasp/action/PickObject`
- `/manipulation/place`：`tracer_piper_color_grasp/action/PlaceObject`

PICK 检查停车、识别颜色、预抓取、下降、闭爪、抬升，再进入 `carry_joint_values` 并保持闭爪。
PLACE 检查停车且携物，读取执行瞬间真实的
`map -> piper_base_link` TF，把固定箱心转到机械臂坐标系，
再用 MoveIt 使两指中心到达箱心正上方、松爪并清除规划附着。
`return_home=true` 时回到 `home_joint_values`。PLACE 不识别颜色，也不执行车载托盘旋转。
一次只接受一个 action；携物时拒绝新 PICK，无物块时拒绝 PLACE。

```bash
cd tracer_navigate_w_grab
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 action send_goal --feedback /manipulation/pick \
  tracer_piper_color_grasp/action/PickObject '{color: red}'
ros2 action send_goal --feedback /manipulation/place \
  tracer_piper_color_grasp/action/PlaceObject '{return_home: true}'
```

红色、黄色、绿色分别传 `red`、`yellow`、`green`。
完整两轮任务使用 `tracer_pick_place_system.launch.py` 和 `/pick_place_mission/start`，
详见 `tracer_pick_place_mission/README.md`。单独启动本包：

```bash
ros2 launch tracer_piper_color_grasp color_grasp_sim.launch.py
# 仿真与 MoveIt 已运行时：
ros2 launch tracer_piper_color_grasp color_grasp_sim.launch.py start_stack:=false
```

配置位于 `config/grasp_task.yaml`，包括 HSV、RGB-D 稳定帧、抓取偏置、工作空间、关节姿态、
轨迹速度、开闭爪参数以及放置参数。`place.basket_center_x_m/y_m`
是 SDF 中储物箱几何中心 `map=(-4.0,4.0)`，`place.finger_center_z_m=0.325`
是两指中心相对 `piper_base_link` 的释放高度。放置时保留已验证的
夹爪方向，X/Y 根据实际停车 TF 现算，不使用理想导航点写死。

```bash
ros2 topic echo /color_grasp/state
ros2 topic echo /color_grasp/busy --qos-durability transient_local
ros2 topic echo /manipulation/has_payload --qos-durability transient_local
ros2 run rqt_image_view rqt_image_view /wrist_camera/image
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool '{data: true}'
```

软件急停、action 取消或执行期间底盘运动会停止机械臂。闭爪之后失败不会自动松爪。
`has_payload` 是软件状态，当前没有接触传感器或真实夹持成功反馈；规划附着不等于 Gazebo
物理连接。须观察真实物块是否抬升并随车运输，才能确认物理任务完成。

2.0 精简版只生成并提供 `PickObject.action` 和 `PlaceObject.action`。
