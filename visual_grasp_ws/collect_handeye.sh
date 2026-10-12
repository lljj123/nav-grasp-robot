#!/usr/bin/env bash
set -eo pipefail

workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$workspace_root/.." && pwd)"
if [[ ! -t 0 ]]; then
    echo "请在交互终端前台运行本脚本。" >&2
    exit 2
fi
source "$repo_root/scripts/project_env.bash"
source "$ROS_SETUP"
source "$workspace_root/install/setup.bash"
set -u
exec ros2 run visual_grasp_calibration handeye_collect --ros-args "$@"
