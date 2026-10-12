#!/usr/bin/env bash
set -eo pipefail

workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$workspace_root/.." && pwd)"
source "$repo_root/scripts/project_env.bash"

# Only one perception/execution stack may own the fixed ROS node and service
# names at a time.  Two stacks with different allow_robot_commands values make
# a service call nondeterministically reach either executor.
lock_file="/tmp/visual_grasp_perception_${ROS_DOMAIN_ID:-0}.lock"
exec 9>"$lock_file"
if ! flock -n 9; then
    echo "错误：已有 Visual Grasp 感知栈正在运行。" >&2
    echo "请先在原启动终端按 Ctrl-C，再重新运行本脚本。" >&2
    exit 1
fi

launch_pattern='python3 .*ros2 launch visual_grasp_bringup perception.launch.py'
mapfile -t existing_launch_pids < <(pgrep -f -- "$launch_pattern" || true)
if ((${#existing_launch_pids[@]} > 0)); then
    echo "错误：检测到旧的 Visual Grasp launch 进程：${existing_launch_pids[*]}" >&2
    echo "请先在对应终端按 Ctrl-C，确认旧节点退出后再启动。" >&2
    exit 1
fi

source "$ROS_SETUP"
nav_grasp_source_if_present "${ARM_DRIVER_SETUP:-}"
source "$workspace_root/install/setup.bash"
set -u
launch_args=()
if [[ -n "${NAV_GRASP_CALIBRATION_DIR:-}" ]]; then
    launch_args+=("calibration_dir:=$NAV_GRASP_CALIBRATION_DIR")
fi
if [[ -n "${NAV_GRASP_GRIPPER_REPORT:-}" ]]; then
    launch_args+=("gripper_range_report_path:=$NAV_GRASP_GRIPPER_REPORT")
fi
exec ros2 launch visual_grasp_bringup perception.launch.py \
    "${launch_args[@]}" "$@"
