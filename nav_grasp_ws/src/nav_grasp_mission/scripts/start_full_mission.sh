#!/usr/bin/env bash
# Managed one-command launcher for navigation, arm, perception, and mission.

set -Eeuo pipefail

LAUNCHER_VERSION="2026-10-13-portable-v5"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
NAV_GRASP_ROOT="$(cd -- "$script_dir/../../../.." && pwd)"
source "$NAV_GRASP_ROOT/scripts/project_env.bash"
NAV_WS="$SLAM_WS"

BASE_LIDAR_LAUNCH="${BASE_LIDAR_LAUNCH:-$NAV_WS/scripts/base_lidar.launch.py}"
MISSION_CONFIG="${NAV_GRASP_MISSION_CONFIG:-$MISSION_WS/src/nav_grasp_mission/config/mission.yaml}"
DEFAULT_MAP="${NAV_GRASP_MAP:-$NAV_WS/maps/map.yaml}"
MAP_FILE="$DEFAULT_MAP"
DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
LOCK_FILE="/tmp/nav_grasp_full_mission_${DOMAIN_ID}.lock"

green='\033[0;32m'
yellow='\033[1;33m'
red='\033[0;31m'
reset='\033[0m'

info() { printf '%b[INFO]%b %s\n' "$green" "$reset" "$*"; }
warn() { printf '%b[WARN]%b %s\n' "$yellow" "$reset" "$*" >&2; }
die() { printf '%b[ERROR]%b %s\n' "$red" "$reset" "$*" >&2; exit 1; }

usage() {
    printf '%s\n' \
        "用法：$0 [--map /absolute/path/map.yaml] [--mission-config /absolute/path/mission.yaml]" \
        '' \
        "  --map             地图 YAML，默认读取 NAV_GRASP_MAP 或 $DEFAULT_MAP" \
        "  --mission-config  任务参数，默认读取 NAV_GRASP_MISSION_CONFIG"
}

while (($# > 0)); do
    case "$1" in
        --map)
            (($# >= 2)) || die '--map 后缺少地图路径'
            MAP_FILE="$2"
            shift 2
            ;;
        --mission-config)
            (($# >= 2)) || die '--mission-config 后缺少参数文件路径'
            MISSION_CONFIG="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            usage >&2
            die "未知参数：$1"
            ;;
    esac
done

require_file() {
    [[ -f "$1" ]] || die "缺少文件：$1"
}

require_executable() {
    [[ -x "$1" ]] || die "文件不存在或不可执行：$1"
}

require_file "$MAP_FILE"
require_file "$BASE_LIDAR_LAUNCH"
require_file "$ROS_SETUP"
require_file "$NAV_WS/setup.bash"
require_file "$VISUAL_WS/install/setup.bash"
require_file "$MISSION_WS/install/setup.bash"
require_file "$MISSION_CONFIG"
require_executable "$VISUAL_WS/start_arm_control.sh"
require_executable "$VISUAL_WS/start_perception.sh"
require_executable "${ARM_CONTROL_SCRIPT:-}"
require_executable "${CAMERA_START_SCRIPT:-}"
require_executable "${CAMERA_STOP_SCRIPT:-}"
require_executable "${CAN_SETUP_SCRIPT:-}"

set +u
source "$ROS_SETUP"
nav_grasp_source_if_present "${ARM_DRIVER_SETUP:-}"
set -u
command -v ros2 >/dev/null 2>&1 || die '未找到 ros2'
command -v gnome-terminal >/dev/null 2>&1 || die '未找到 gnome-terminal'
command -v flock >/dev/null 2>&1 || die '未找到 flock'
command -v pgrep >/dev/null 2>&1 || die '未找到 pgrep'

target_summary="$({ python3 - "$MISSION_CONFIG" <<'PY'
import math
import sys

import yaml

config = yaml.safe_load(open(sys.argv[1], encoding='utf-8'))
params = config['nav_grasp_mission']['ros__parameters']
if params.get('target_pose_configured') is not True:
    raise SystemExit('target_pose_configured 不是 true')
pose = params.get('target_pose_xyyaw')
if not isinstance(pose, list) or len(pose) != 3:
    raise SystemExit('target_pose_xyyaw 必须是 [x, y, yaw]')
values = [float(value) for value in pose]
if not all(math.isfinite(value) for value in values):
    raise SystemExit('target_pose_xyyaw 含非有限数值')
print(f'x={values[0]:.3f}, y={values[1]:.3f}, yaw={values[2]:.3f} rad')
PY
} 2>&1)" || die "任务目标配置无效：$target_summary"

info "启动器版本：$LAUNCHER_VERSION"
info "地图：$MAP_FILE"
info "抓取停车位：$target_summary"

[[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]] || \
    die '当前没有图形桌面，无法打开 RViz 和组件终端'

exec 9>"$LOCK_FILE"
flock -n 9 || die '已有一个完整任务启动器正在运行'

RUN_DIR="$(mktemp -d /tmp/nav_grasp_full_mission.XXXXXX)"
MAIN_PGID="$(ps -o pgid= -p "$$" | tr -d ' ')"
declare -a COMPONENT_NAMES=()
declare -a COMPONENT_PIDS=()
declare -a COMPONENT_PGIDS=()
declare -a TERMINAL_PIDS=()
SHUTTING_DOWN=false

set +u
source "$NAV_WS/setup.bash"
nav_grasp_source_if_present "${ARM_DRIVER_SETUP:-}"
source "$VISUAL_WS/install/setup.bash"
source "$MISSION_WS/install/setup.bash"
set -u

group_signal() {
    local signal="$1"
    local pid="$2"
    local pgid="$3"
    if [[ "$pgid" =~ ^[0-9]+$ && "$pgid" != "$MAIN_PGID" ]]; then
        kill "-$signal" -- "-$pgid" 2>/dev/null || true
    elif [[ "$pid" =~ ^[0-9]+$ ]]; then
        kill "-$signal" "$pid" 2>/dev/null || true
    fi
}

any_component_alive() {
    local pid
    for pid in "${COMPONENT_PIDS[@]:-}"; do
        if kill -0 "$pid" 2>/dev/null; then
            return 0
        fi
    done
    return 1
}

pick_state_is_motion_free() {
    grep -Eq '(IDLE|AWAITING_CONFIRMATION|SUCCEEDED|ERROR):' <<<"$1"
}

safe_stop_ros_activity() {
    local mission_status pick_status arm_status mission_was_running=false
    mission_status="$(timeout 3 ros2 service call \
        /nav_grasp/status std_srvs/srv/Trigger '{}' 2>/dev/null || true)"
    if grep -q 'running=True' <<<"$mission_status"; then
        mission_was_running=true
        warn '检测到任务仍在运行：先取消任务并停止底盘'
        timeout 3 ros2 service call \
            /nav_grasp/cancel std_srvs/srv/Trigger '{}' \
            >/dev/null 2>&1 || true
    fi
    timeout 3 ros2 topic pub --once \
        /cmd_vel geometry_msgs/msg/Twist '{}' \
        >/dev/null 2>&1 || true

    pick_status="$(timeout 3 ros2 service call \
        /visual_grasp/pick_status std_srvs/srv/Trigger '{}' \
        2>/dev/null || true)"
    if { [[ -n "$pick_status" ]] && ! pick_state_is_motion_free "$pick_status"; } \
            || { [[ -z "$pick_status" ]] \
                && [[ "$mission_was_running" == true ]]; }; then
        warn '机械臂可能仍在抓取动作中：先调用软件急停'
        timeout 3 ros2 service call \
            /arm/emergency_stop std_srvs/srv/Empty '{}' \
            >/dev/null 2>&1 || true
        return
    fi

    arm_status="$(timeout 3 ros2 topic echo --once \
        /arm/feedback/arm_status 2>/dev/null || true)"
    if grep -Eq 'motion_status:[[:space:]]*[1-9]' <<<"$arm_status"; then
        warn '机械臂反馈为运动中：先调用软件急停'
        timeout 3 ros2 service call \
            /arm/emergency_stop std_srvs/srv/Empty '{}' \
            >/dev/null 2>&1 || true
    fi
}

cleanup() {
    local original_status=$?
    if [[ "$SHUTTING_DOWN" == true ]]; then
        return
    fi
    SHUTTING_DOWN=true
    trap - EXIT INT TERM HUP

    if ((${#COMPONENT_PIDS[@]} > 0)); then
        printf '\n'
        warn '正在安全停止完整项目，请勿关闭电源……'
        safe_stop_ros_activity

        local index
        for ((index=${#COMPONENT_PIDS[@]} - 1; index >= 0; index--)); do
            info "停止${COMPONENT_NAMES[$index]}"
            group_signal INT \
                "${COMPONENT_PIDS[$index]}" "${COMPONENT_PGIDS[$index]}"
        done

        local deadline=$((SECONDS + 8))
        while any_component_alive && ((SECONDS < deadline)); do
            sleep 0.2
        done
        if any_component_alive; then
            warn '部分组件未及时退出，发送 TERM'
            for ((index=${#COMPONENT_PIDS[@]} - 1; index >= 0; index--)); do
                group_signal TERM \
                    "${COMPONENT_PIDS[$index]}" "${COMPONENT_PGIDS[$index]}"
            done
            sleep 1
        fi
        if any_component_alive; then
            warn '仍有组件未退出，执行最终强制清理'
            for ((index=${#COMPONENT_PIDS[@]} - 1; index >= 0; index--)); do
                group_signal KILL \
                    "${COMPONENT_PIDS[$index]}" "${COMPONENT_PGIDS[$index]}"
            done
        fi

        "$CAMERA_STOP_SCRIPT" >/dev/null 2>&1 || true
        local terminal_pid terminal_deadline
        for terminal_pid in "${TERMINAL_PIDS[@]:-}"; do
            terminal_deadline=$((SECONDS + 3))
            while kill -0 "$terminal_pid" 2>/dev/null \
                    && ((SECONDS < terminal_deadline)); do
                sleep 0.1
            done
            if kill -0 "$terminal_pid" 2>/dev/null; then
                kill -TERM "$terminal_pid" 2>/dev/null || true
            fi
            wait "$terminal_pid" 2>/dev/null || true
        done
        info '所有项目组件和终端均已关闭'
    fi

    if [[ -n "${RUN_DIR:-}" && -d "$RUN_DIR" ]]; then
        find "$RUN_DIR" -depth -delete 2>/dev/null || true
    fi
    exit "$original_status"
}

trap cleanup EXIT
trap 'exit 130' INT TERM HUP

topic_has_publishers() {
    local output
    output="$(timeout 3 ros2 topic info "$1" 2>/dev/null || true)"
    grep -Eq 'Publisher count: [1-9][0-9]*' <<<"$output"
}

collect_existing_conflicts() {
    local actions services name
    actions="$(timeout 5 ros2 action list 2>/dev/null || true)"
    services="$(timeout 5 ros2 service list 2>/dev/null || true)"
    conflicts=()
    for name in \
        /odom \
        /scan \
        /camera/camera/color/image_raw \
        /visual_grasp/execution_state \
        /nav_grasp/state; do
        if topic_has_publishers "$name"; then
            conflicts+=("$name")
        fi
    done
    if grep -Fxq -- /navigate_to_pose <<<"$actions"; then
        conflicts+=(/navigate_to_pose)
    fi
    for name in /visual_grasp/plan_pick /nav_grasp/start; do
        if grep -Fxq -- "$name" <<<"$services"; then
            conflicts+=("$name")
        fi
    done
}

stop_existing_pattern() {
    local label="$1"
    local pattern="$2"
    local -a pids=()
    local -a pgids=()
    local pid pgid signal deadline alive
    local -A seen_pgids=()

    mapfile -t pids < <(pgrep -f -- "$pattern" 2>/dev/null || true)
    ((${#pids[@]} > 0)) || return 0
    for pid in "${pids[@]}"; do
        pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')"
        if [[ "$pgid" =~ ^[0-9]+$ && "$pgid" != "$MAIN_PGID" \
                && -z "${seen_pgids[$pgid]:-}" ]]; then
            seen_pgids[$pgid]=1
            pgids+=("$pgid")
        fi
    done
    ((${#pgids[@]} > 0)) || return 0

    warn "停止旧${label}进程组：${pgids[*]}"
    for signal in INT TERM KILL; do
        for pgid in "${pgids[@]}"; do
            kill "-$signal" -- "-$pgid" 2>/dev/null || true
        done
        [[ "$signal" == INT ]] && deadline=$((SECONDS + 8)) || \
            deadline=$((SECONDS + 2))
        while ((SECONDS < deadline)); do
            alive=false
            for pgid in "${pgids[@]}"; do
                if kill -0 -- "-$pgid" 2>/dev/null; then
                    alive=true
                    break
                fi
            done
            [[ "$alive" == false ]] && return 0
            sleep 0.2
        done
    done
    return 0
}

collect_existing_conflicts
if ((${#conflicts[@]} > 0)); then
    warn "检测到已有任务组件：${conflicts[*]}，正在安全接管并清理"
    safe_stop_ros_activity
fi

visual_ws_pattern="$(python3 -c 'import re,sys; print(re.escape(sys.argv[1]))' "$VISUAL_WS")"
nav_ws_pattern="$(python3 -c 'import re,sys; print(re.escape(sys.argv[1]))' "$NAV_WS")"
mission_ws_pattern="$(python3 -c 'import re,sys; print(re.escape(sys.argv[1]))' "$MISSION_WS")"
base_lidar_pattern="$(python3 -c 'import re,sys; print(re.escape(sys.argv[1]))' "$BASE_LIDAR_LAUNCH")"

stop_existing_pattern '任务状态机' \
    '^/usr/bin/python3 .*ros2 launch nav_grasp_mission mission\.launch\.py( |$)'
stop_existing_pattern '视觉抓取' \
    '^/usr/bin/python3 .*ros2 launch visual_grasp_bringup perception\.launch\.py( |$)'
stop_existing_pattern '视觉抓取残留节点' \
    "^(/usr/bin/python3 )?${visual_ws_pattern}/install/visual_grasp_"
stop_existing_pattern 'Nav2' \
    '^/usr/bin/python3 .*ros2 launch nav navigation\.launch\.py( |$)'
stop_existing_pattern '底盘与雷达' \
    "^/usr/bin/python3 .*ros2 launch ${base_lidar_pattern}( |$)"
stop_existing_pattern '导航残留节点' \
    "^(/usr/bin/python3 )?${nav_ws_pattern}/install/"
stop_existing_pattern 'D435相机' \
    '^/usr/bin/python3 .*ros2 launch realsense2_camera rs_launch\.py( |$)'
"$CAMERA_STOP_SCRIPT" >/dev/null 2>&1 || true
if [[ -n "${ARM_DRIVER_WS:-}" ]]; then
    arm_ws_pattern="$(python3 -c 'import re,sys; print(re.escape(sys.argv[1]))' "$ARM_DRIVER_WS")"
    stop_existing_pattern '机械臂驱动' \
        "^(/usr/bin/python3 )?${arm_ws_pattern}/install/agx_arm_ctrl/lib/agx_arm_ctrl/agx_arm_ctrl_single( |$)"
fi
stop_existing_pattern '状态机残留节点' \
    "^(/usr/bin/python3 )?${mission_ws_pattern}/install/nav_grasp_mission/"

for _ in {1..10}; do
    collect_existing_conflicts
    ((${#conflicts[@]} == 0)) && break
    sleep 0.5
done
if ((${#conflicts[@]} > 0)); then
    die "旧组件未能完全退出：${conflicts[*]}。请查看对应旧终端。"
fi
info '旧任务组件清理完成，可以启动新任务'

configure_chassis_can() {
    local iface="${BASE_CAN_INTERFACE:-can1}"
    local bitrate="${BASE_CAN_BITRATE:-500000}"
    [[ -e "/sys/class/net/$iface" ]] || die "未找到底盘 CAN 接口 $iface"
    if ip -details link show "$iface" 2>/dev/null | grep -q 'BUS-OFF' \
            || ! ip link show "$iface" | grep -qw UP; then
        info "配置底盘 CAN $iface（$bitrate bit/s）"
        sudo ip link set "$iface" down 2>/dev/null || true
        sudo ip link set "$iface" type can bitrate "$bitrate" restart-ms 100
        sudo ip link set "$iface" up
    else
        info "底盘 CAN $iface 已启用"
    fi
}

info '检查 sudo 凭据和两个 CAN 接口'
sudo -v
configure_chassis_can
"$CAN_SETUP_SCRIPT"

open_terminal() {
    local key="$1"
    local title="$2"
    local command="$3"
    local pid_file="$RUN_DIR/$key.pid"
    local wrapped
    printf -v wrapped \
        'pid=$$; pgid=$(ps -o pgid= -p "$$" | tr -d " "); printf "%%s %%s\\n" "$pid" "$pgid" > %q; %s' \
        "$pid_file" "$command"

    gnome-terminal --wait --title="$title" -- bash -lc "$wrapped" \
        >/dev/null 2>&1 &
    local terminal_pid=$!
    local deadline=$((SECONDS + 10))
    while [[ ! -s "$pid_file" ]] && ((SECONDS < deadline)); do
        kill -0 "$terminal_pid" 2>/dev/null || die "无法打开${title}终端"
        sleep 0.1
    done
    [[ -s "$pid_file" ]] || die "${title}终端未报告进程 PID"

    local component_pid component_pgid
    read -r component_pid component_pgid < "$pid_file"
    [[ "$component_pid" =~ ^[0-9]+$ ]] || die "${title}返回了无效 PID"
    [[ "$component_pgid" =~ ^[0-9]+$ ]] || die "${title}返回了无效 PGID"
    kill -0 "$component_pid" 2>/dev/null || die "${title}启动后立即退出"
    COMPONENT_NAMES+=("$title")
    COMPONENT_PIDS+=("$component_pid")
    COMPONENT_PGIDS+=("$component_pgid")
    TERMINAL_PIDS+=("$terminal_pid")
}

all_components_alive() {
    local index
    for ((index=0; index<${#COMPONENT_PIDS[@]}; index++)); do
        if ! kill -0 "${COMPONENT_PIDS[$index]}" 2>/dev/null; then
            warn "${COMPONENT_NAMES[$index]}已意外退出"
            return 1
        fi
    done
    return 0
}

wait_until() {
    local label="$1"
    local timeout_s="$2"
    shift 2
    local deadline=$((SECONDS + timeout_s))
    info "等待${label}……"
    while ((SECONDS < deadline)); do
        if "$@"; then
            info "${label}已就绪"
            return 0
        fi
        all_components_alive || die "等待${label}时有组件退出"
        sleep 1
    done
    die "等待${label}超时；请查看刚打开的组件终端"
}

topic_set_contains() {
    local output name
    output="$(timeout 5 ros2 topic list 2>/dev/null || true)"
    for name in "$@"; do
        grep -Fxq -- "$name" <<<"$output" || return 1
    done
}

service_set_contains() {
    local output name
    output="$(timeout 5 ros2 service list 2>/dev/null || true)"
    for name in "$@"; do
        grep -Fxq -- "$name" <<<"$output" || return 1
    done
}

navigation_ready() {
    local output
    output="$(timeout 5 ros2 action list 2>/dev/null || true)"
    grep -Fxq -- /navigate_to_pose <<<"$output"
}

localization_ready() {
    local output
    output="$(timeout 4 ros2 run tf2_ros tf2_echo map base_link \
        2>/dev/null || true)"
    grep -q 'Translation:' <<<"$output"
}

printf -v map_arg '%q' "$MAP_FILE"
printf -v mission_config_arg '%q' "$MISSION_CONFIG"

open_terminal base '1-底盘与雷达' \
    "source '$NAV_WS/setup.bash'; exec ros2 launch '$BASE_LIDAR_LAUNCH' port_name:='${BASE_CAN_INTERFACE:-can1}'"
wait_until '底盘与雷达（/odom、/scan）' 45 \
    topic_set_contains /odom /scan

open_terminal camera '2-D435相机' \
    "exec '$CAMERA_START_SCRIPT'"
wait_until 'D435 相机' 45 \
    topic_set_contains "${CAMERA_IMAGE_TOPIC:-/camera/camera/color/image_raw}"

open_terminal navigation '3-Nav2与RViz' \
    "source '$NAV_WS/setup.bash'; exec ros2 launch nav navigation.launch.py map:=$map_arg"
wait_until 'Nav2（/navigate_to_pose）' 60 navigation_ready

open_terminal arm '4-机械臂驱动' \
    "exec '$VISUAL_WS/start_arm_control.sh' --replace"
wait_until '机械臂反馈' 60 \
    topic_set_contains /arm/feedback/arm_status \
    /arm/feedback/joint_states /arm/feedback/gripper_status

open_terminal perception '5-视觉抓取' \
    "exec '$VISUAL_WS/start_perception.sh' allow_robot_commands:=true"
wait_until '视觉抓取接口' 60 \
    service_set_contains /visual_grasp/plan_pick \
    /visual_grasp/execute_full_pick

open_terminal mission '6-任务状态机' \
    "source '$ROS_SETUP'; source '$MISSION_WS/install/setup.bash'; exec ros2 launch nav_grasp_mission mission.launch.py params_file:=$mission_config_arg"
wait_until '任务状态机' 30 \
    service_set_contains /nav_grasp/start /nav_grasp/status

printf '\n%b所有组件已启动。%b\n' "$green" "$reset"
printf '%s\n' \
    '1. 在 RViz 点击 “2D Pose Estimate”。' \
    '2. 标出机器人当前真实位置和朝向。' \
    '3. 确认激光轮廓与地图重合，机器人位于希望返航的位置。' \
    '4. 确认路线、抓取区和机械臂工作空间安全。' \
    '主控终端将保持运行；按 Ctrl-C 会安全停止并关闭全部组件终端。'

while true; do
    read -rp '完成定位和安全检查后按回车开始任务（Ctrl-C 暂不启动）... ' _
    all_components_alive || die '有组件已经退出，不能开始任务'
    if localization_ready; then
        break
    fi
    warn '尚未检测到 map -> base_link，请重新执行 2D Pose Estimate'
done

info '定位有效，正在启动导航—抓取—返航任务'
start_response="$(ros2 service call \
    /nav_grasp/start std_srvs/srv/Trigger '{}')"
printf '%s\n' "$start_response"
grep -q 'success=True' <<<"$start_response" || \
    die '任务状态机拒绝启动，请查看状态机终端信息'

printf '\n%s\n' \
    '任务已经提交，主控正在监视任务状态。' \
    '按 Ctrl-C 会先取消任务；若机械臂仍在动作，会调用软件急停。' \
    '实体危险情况下，仍应优先使用机械臂实体急停。'

mission_status_text() {
    local output
    output="$(timeout 4 ros2 service call \
        /nav_grasp/status std_srvs/srv/Trigger '{}' 2>/dev/null || true)"
    sed -n "s/.*message='\(.*\)'.*/\1/p" <<<"$output" | tail -n 1
}

state_label() {
    case "$1" in
        WAIT_DEPENDENCIES) printf '等待全部依赖' ;;
        CAPTURE_START_POSE) printf '记录返航起点' ;;
        NAVIGATE_TO_TARGET) printf '导航到抓取点' ;;
        WAIT_BASE_STILL) printf '等待底盘稳定' ;;
        PLAN_PICK) printf '15秒内持续检测抓取目标' ;;
        EXECUTE_PICK) printf '启动完整抓取' ;;
        WAIT_PICK_RESULT) printf '机械臂抓取中' ;;
        RETURN_TO_START) printf '返回任务起点' ;;
        DONE) printf '任务完成' ;;
        ERROR) printf '任务错误' ;;
        CANCELED) printf '任务已取消' ;;
        *) printf '%s' "$1" ;;
    esac
}

previous_status=''
missing_status_count=0
terminal_state=''
while true; do
    all_components_alive || die '有组件意外退出，正在关闭完整项目'
    current_status="$(mission_status_text)"
    if [[ -z "$current_status" ]]; then
        missing_status_count=$((missing_status_count + 1))
        if ((missing_status_count >= 3)); then
            die '连续无法读取任务状态，正在关闭完整项目'
        fi
        sleep 1
        continue
    fi
    missing_status_count=0
    current_state="${current_status#state=}"
    current_state="${current_state%%;*}"
    if [[ "$current_status" != "$previous_status" ]]; then
        info "任务状态：$(state_label "$current_state") | $current_status"
        previous_status="$current_status"
    fi
    if [[ "$current_status" == *'running=False'* ]]; then
        case "$current_state" in
            DONE|ERROR|CANCELED)
                terminal_state="$current_state"
                break
                ;;
        esac
    fi
    sleep 1
done

case "$terminal_state" in
    DONE) info '导航、抓取和返航任务已完成' ;;
    ERROR) warn '任务以 ERROR 结束，请先查看各组件终端日志' ;;
    CANCELED) warn '任务已取消' ;;
esac
read -rp '按回车关闭本项目启动的全部进程和终端... ' _
