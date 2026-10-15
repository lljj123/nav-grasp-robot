#!/usr/bin/env bash
set -eo pipefail

workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$workspace_root/../../.." && pwd)"
source "$repo_root/scripts/project_env.bash"
source "$ROS_SETUP"
nav_grasp_source_if_present "${ARM_DRIVER_SETUP:-}"
source "$workspace_root/install/setup.bash"
set -u

calibrator_executable="$workspace_root/install/visual_grasp_calibration/lib/visual_grasp_calibration/handeye_calibrator"
bridge_executable="$workspace_root/install/visual_grasp_calibration/lib/visual_grasp_calibration/piper_pose_tf_bridge"
if pgrep -f "^/usr/bin/python3 (${calibrator_executable}|${bridge_executable})( |$)" \
        >/dev/null; then
    echo "已有手眼标定节点运行。请先关闭原采样终端，或运行 ./stop_handeye.sh。" >&2
    exit 1
fi
launch_args=()
if [[ -n "${NAV_GRASP_CALIBRATION_DIR:-}" ]]; then
    launch_args+=("calibration_dir:=$NAV_GRASP_CALIBRATION_DIR")
fi
exec ros2 launch visual_grasp_calibration handeye_calibration.launch.py \
    "${launch_args[@]}" "$@"
