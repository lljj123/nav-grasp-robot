#!/usr/bin/env bash
# =============================================================================
# SLAM / Nav2 一键启动脚本（默认硬件配置：Tracer + MID-360）
#
# 用法：
#   ./scripts/start.sh
#
# 模式 1 建图：
#   - 后台：底盘 + MID-360 + /scan 转换（日志 logs/base_lidar.log）
#   - 新终端：SLAM Toolbox 建图（自动弹出 RViz 建图画面）
#   - 新终端：键盘遥控底盘（限速 0.3 m/s / 0.5 rad/s）
#   - 本终端：控制菜单（保存地图 / 重开遥控 / 查看日志 / 全部停止）
#
# 模式 2 导航：
#   - 后台：底盘 + MID-360 + /scan 转换
#   - 新终端：Nav2 导航（自动弹出 RViz，选择已保存的地图）
#   - 本终端：控制菜单（重启导航 / 换地图 / 查看日志 / 全部停止）
# =============================================================================
set -uo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
NAV_GRASP_ROOT="$(cd -- "$script_dir/../../../.." && pwd)"
source "$NAV_GRASP_ROOT/scripts/project_env.bash"
WS="$SLAM_WS"
LOG_DIR="$WS/logs"
BASE_LIDAR_LAUNCH="$WS/scripts/base_lidar.launch.py"
CAN_PORT="${BASE_CAN_INTERFACE:-can1}"
CAN_BITRATE="${BASE_CAN_BITRATE:-500000}"
TELEOP_SPEED="0.3"
TELEOP_TURN="0.5"
SELECTED_MAP=""

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
info() { echo -e "${GREEN}[INFO]${NC} $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
err()  { echo -e "${RED}[ERROR]${NC} $*"; }

# ---------------------------------------------------------------- 环境检查
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    err "未检测到图形显示环境，请在桌面会话中运行本脚本"
    exit 1
fi
if ! command -v gnome-terminal >/dev/null 2>&1 \
    && ! command -v x-terminal-emulator >/dev/null 2>&1; then
    err "未找到图形终端 (gnome-terminal)，请先安装"
    exit 1
fi
if ! [[ -f "$WS/setup.bash" ]]; then
    err "未找到 $WS/setup.bash，请先编译：cd $WS && ./build.sh"
    exit 1
fi
# shellcheck disable=SC1091
source "$WS/setup.bash"

# ---------------------------------------------------------------- 基础函数
open_term() {  # $1=标题  $2=bash 命令串
    if command -v gnome-terminal >/dev/null 2>&1; then
        gnome-terminal --title="$1" -- bash -c "$2" >/dev/null 2>&1 &
    else
        x-terminal-emulator -T "$1" -e bash -c "$2" >/dev/null 2>&1 &
    fi
}

setup_can() {
    if ! ip link show "$CAN_PORT" &>/dev/null; then
        warn "未找到 $CAN_PORT 接口，请确认 CAN 硬件已连接"
        read -rp "仍要继续吗? [y/N] " a
        [[ "$a" == "y" || "$a" == "Y" ]]
        return
    fi
    # BUS-OFF 恢复：控制器脱离总线后必须重建接口；restart-ms 让以后自动恢复
    if ip -details link show "$CAN_PORT" 2>/dev/null | grep -q 'BUS-OFF' \
        || ! ip link show "$CAN_PORT" | grep -qw UP; then
        info "重置 $CAN_PORT（${CAN_BITRATE} bit/s，BUS-OFF 自动恢复已启用）"
        sudo ip link set "$CAN_PORT" down || true
        if sudo ip link set "$CAN_PORT" type can bitrate "$CAN_BITRATE" restart-ms 100 \
            && sudo ip link set "$CAN_PORT" up; then
            info "$CAN_PORT 重置成功"
        else
            err "CAN 配置失败"
            return 1
        fi
    else
        info "$CAN_PORT 已启动"
    fi
}

check_chassis_alive() {  # 底盘 CAN 是否在正常通信（日志无持续发送失败）
    local log="$LOG_DIR/base_lidar.log"
    [[ -f "$log" ]] || return 0
    local tail_lines; tail_lines=$(tail -n 15 "$log" 2>/dev/null)
    if grep -q "Failed to send CAN frame" <<< "$tail_lines"; then
        warn "底盘 CAN 通信失败（Failed to send CAN frame）：机器人不会动！"
        warn "请检查：底盘是否上电 / CAN 线 H-L 是否接好 / 接口是否 BUS-OFF"
        read -rp "处理完成后重试? [Y/n] " a
        [[ "$a" != "n" && "$a" != "N" ]] || return 1
        setup_can || return 1
    fi
}

start_base_lidar() {
    # 已有底盘在跑则跳过，避免重复启动
    if timeout 5 ros2 topic list 2>/dev/null | grep -q '^/odom$'; then
        warn "检测到 /odom 已存在，跳过底盘/雷达启动（可能已在运行）"
        return 0
    fi
    mkdir -p "$LOG_DIR"
    info "后台启动底盘 + 激光雷达（日志：$LOG_DIR/base_lidar.log）"
    nohup setsid bash -c "source '$WS/setup.bash'; exec ros2 launch '$BASE_LIDAR_LAUNCH' port_name:=$CAN_PORT" \
        >> "$LOG_DIR/base_lidar.log" 2>&1 &
    info "等待雷达数据 /scan ..."
    for _ in $(seq 1 20); do
        if timeout 3 ros2 topic list 2>/dev/null | grep -q '^/scan$'; then
            info "底盘 + 雷达就绪"
            check_chassis_alive || return 1
            return 0
        fi
        sleep 1
    done
    warn "20 秒内未见 /scan，可另开终端执行: tail -f $LOG_DIR/base_lidar.log"
    read -rp "继续吗? [Y/n] " a
    [[ "$a" != "n" && "$a" != "N" ]]
}

stop_teleop()      { pkill -INT -f 'teleop_twist_keyboard' 2>/dev/null || true; }
stop_mapping()     { pkill -INT -f 'slam mapping.launch.py' 2>/dev/null || true; }
stop_nav() {
    pkill -INT -f 'navigation.launch.py' 2>/dev/null || true
    pkill -INT -f 'nav2_bringup' 2>/dev/null || true
}
stop_base_lidar()  { pkill -INT -f 'base_lidar.launch.py' 2>/dev/null || true; }

start_camera() {
    # 相机已运行则跳过
    local image_topic="${CAMERA_IMAGE_TOPIC:-/camera/camera/color/image_raw}"
    if timeout 5 ros2 topic list 2>/dev/null | grep -Fxq "$image_topic"; then
        warn "检测到相机图像话题已存在，跳过相机启动"
        return 0
    fi
    if [[ -z "${CAMERA_START_SCRIPT:-}" || ! -x "$CAMERA_START_SCRIPT" ]]; then
        warn "未配置 CAMERA_START_SCRIPT，跳过相机"
        return 0
    fi
    mkdir -p "$LOG_DIR"
    info "后台启动 D435 相机（日志：$LOG_DIR/camera.log）"
    nohup setsid "$CAMERA_START_SCRIPT" >> "$LOG_DIR/camera.log" 2>&1 &
    info "等待相机图像 $image_topic ..."
    for _ in $(seq 1 20); do
        if timeout 3 ros2 topic list 2>/dev/null | grep -Fxq "$image_topic"; then
            info "相机就绪"
            return 0
        fi
        sleep 1
    done
    warn "20 秒内未见相机图像，可查看: tail -f $LOG_DIR/camera.log"
    return 0  # 相机失败不阻断建图/导航
}

stop_camera() {
    if [[ -n "${CAMERA_STOP_SCRIPT:-}" && -x "$CAMERA_STOP_SCRIPT" ]]; then
        "$CAMERA_STOP_SCRIPT" 2>/dev/null || true
    fi
}

cleanup() {
    echo
    info "停止所有组件 ..."
    stop_camera
    stop_teleop
    stop_mapping
    stop_nav
    stop_base_lidar
    sleep 2
    info "已全部停止"
}
trap cleanup EXIT

# ---------------------------------------------------------------- 建图模式
spawn_teleop() {
    open_term "键盘控制" "source '$WS/setup.bash'; \
echo '=== 键盘遥控底盘（已限速 $TELEOP_SPEED m/s / $TELEOP_TURN rad/s）==='; \
echo '控制/调速按键见下方帮助，Ctrl+C 退出遥控'; \
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard \
--ros-args -p speed:=$TELEOP_SPEED -p turn:=$TELEOP_TURN"
}

save_map_flow() {
    local def="$WS/maps/map" path
    read -rp "保存路径前缀 [回车=$def]: " path
    [[ -n "$path" ]] || path="$def"
    info "保存地图到 ${path}.yaml/.pgm/.posegraph ..."
    if ros2 run slam save_map "$path"; then
        info "保存成功"
    else
        err "保存失败（请确认建图终端仍在运行）"
    fi
}

mode_mapping() {
    info "=== 建图模式 ==="
    open_term "SLAM建图" "cd '$WS/maps'; source '$WS/setup.bash'; \
echo '=== SLAM Toolbox 建图（Ctrl+C 停止建图）==='; \
echo 'RViz 面板 Save Map 按钮会将地图保存到当前目录（即 maps/）'; \
exec ros2 launch slam mapping.launch.py"
    spawn_teleop
    echo
    info "遥控小车慢速行驶建图，建议行驶中完成回环"
    info "RViz 右侧 SlamToolboxPlugin 面板可 Save Map / Save As 保存地图"
    info "用 Save As 时建议保存到 $WS/maps/ 下，导航模式可直接选择"
    while true; do
        echo
        read -rn1 -p "[s]保存地图  [t]重开遥控  [l]底盘日志  [q]停止全部退出 : " ans
        echo
        case "$ans" in
            s|S) save_map_flow ;;
            t|T) spawn_teleop ;;
            l|L) tail -n 30 "$LOG_DIR/base_lidar.log" 2>/dev/null || warn "暂无日志" ;;
            q|Q) break ;;
        esac
    done
}

# ---------------------------------------------------------------- 导航模式
select_map() {
    local maps=() f i=1 def=0 n
    for f in "$WS"/maps/*.yaml; do
        [[ -f "$f" ]] && maps+=("$f")
    done
    if [[ ${#maps[@]} -eq 0 ]]; then
        err "maps/ 下没有地图（*.yaml），请先建图"
        read -rp "或直接输入地图 yaml 完整路径（回车返回）: " n
        if [[ -n "$n" ]]; then
            if [[ -f "$n" ]]; then
                SELECTED_MAP="$n"
                info "使用地图：$SELECTED_MAP"
                return 0
            fi
            err "文件不存在：$n"
        fi
        return 1
    fi
    echo "可用地图："
    for f in "${maps[@]}"; do
        echo "  $i) $(basename "$f")"
        [[ "$(basename "$f")" == "map.yaml" ]] && def=$i
        i=$((i + 1))
    done
    [[ $def -eq 0 ]] && def=1
    read -rp "选择编号 / 直接输入 yaml 路径 [回车=$def]: " n
    [[ -n "$n" ]] || n=$def
    if [[ -f "$n" ]]; then  # 直接输入了路径
        SELECTED_MAP="$n"
    elif [[ "$n" =~ ^[0-9]+$ ]] && (( n >= 1 && n <= ${#maps[@]} )); then
        SELECTED_MAP="${maps[$((n - 1))]}"
    else
        err "无效选择"
        return 1
    fi
    info "使用地图：$SELECTED_MAP"
}

start_nav() {
    open_term "Nav2导航" "source '$WS/setup.bash'; \
echo '=== Nav2 导航 ==='; \
echo 'RViz：先用 2D Pose Estimate 设初始位姿，再用 Nav2 Goal 下发目标'; \
exec ros2 launch nav navigation.launch.py map:='$1'"
}

mode_navigation() {
    info "=== 导航模式 ==="
    select_map || return 1
    start_nav "$SELECTED_MAP"
    while true; do
        echo
        read -rn1 -p "[r]重启导航/换地图  [l]底盘日志  [q]停止全部退出 : " ans
        echo
        case "$ans" in
            r|R)
                stop_nav
                sleep 2
                select_map && start_nav "$SELECTED_MAP"
                ;;
            l|L) tail -n 30 "$LOG_DIR/base_lidar.log" 2>/dev/null || warn "暂无日志" ;;
            q|Q) break ;;
        esac
    done
}

# ---------------------------------------------------------------- 主菜单
main() {
    echo "======================================"
    echo "  Nav-Grasp SLAM & Nav2 启动器"
    echo "--------------------------------------"
    echo "  1) 建图（SLAM Toolbox）"
    echo "  2) 导航（Nav2，使用已保存地图）"
    echo "  3) 退出"
    echo "======================================"
    read -rp "请选择 [1/2/3]: " mode
    case "$mode" in
        1) setup_can && start_base_lidar && start_camera && mode_mapping ;;
        2) setup_can && start_base_lidar && start_camera && mode_navigation ;;
        3) exit 0 ;;
        *) err "无效选择"; exit 1 ;;
    esac
}

main
