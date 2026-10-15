#!/usr/bin/env bash
set -eo pipefail

workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$workspace_root/../../.." && pwd)"
source "$repo_root/scripts/project_env.bash"

lock_file="/tmp/visual_grasp_rviz_${ROS_DOMAIN_ID:-0}.lock"
exec 9>"$lock_file"
if ! flock -n 9; then
    echo "错误：Visual Grasp RViz2 已经在运行。" >&2
    exit 1
fi

source "$ROS_SETUP"
nav_grasp_source_if_present "${ROBOT_MODEL_SETUP:-}"
nav_grasp_source_if_present "${ARM_DRIVER_SETUP:-}"
source "$workspace_root/install/setup.bash"
set -u

exec ros2 launch visual_grasp_bringup visualization.launch.py "$@"
