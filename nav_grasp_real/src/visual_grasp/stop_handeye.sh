#!/usr/bin/env bash
set -eo pipefail

workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
calibrator_executable="$workspace_root/install/visual_grasp_calibration/lib/visual_grasp_calibration/handeye_calibrator"
bridge_executable="$workspace_root/install/visual_grasp_calibration/lib/visual_grasp_calibration/piper_pose_tf_bridge"
pattern="^/usr/bin/python3 (${calibrator_executable}|${bridge_executable})( |$)"

mapfile -t process_ids < <(pgrep -f "$pattern" || true)
if (( ${#process_ids[@]} == 0 )); then
    echo "没有运行中的手眼标定节点。"
    exit 0
fi

echo "仅停止 nav_grasp_real/src/visual_grasp 手眼标定进程：${process_ids[*]}"
kill -TERM "${process_ids[@]}"
for _ in {1..30}; do
    remaining=()
    for process_id in "${process_ids[@]}"; do
        if kill -0 "$process_id" 2>/dev/null; then
            remaining+=("$process_id")
        fi
    done
    if (( ${#remaining[@]} == 0 )); then
        echo "手眼标定节点已停止；相机和机械臂反馈节点未受影响。"
        exit 0
    fi
    sleep 0.1
done

echo "以下标定进程未在3秒内退出：${remaining[*]}" >&2
exit 1
