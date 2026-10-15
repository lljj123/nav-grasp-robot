#!/usr/bin/env bash
set -eo pipefail

workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$workspace_root/../../.." && pwd)"
source "$repo_root/scripts/project_env.bash"
driver_root="${ARM_DRIVER_WS:-}"
driver_setup="${ARM_DRIVER_SETUP:-${driver_root:+$driver_root/install/setup.bash}}"

if [[ -z "$driver_root" || ! -f "$driver_setup" ]]; then
    echo "请在 nav_grasp_real/project.env 中配置 ARM_DRIVER_WS/ARM_DRIVER_SETUP。" >&2
    exit 2
fi
if [[ -z "${CAN_SETUP_SCRIPT:-}" || ! -x "$CAN_SETUP_SCRIPT" ]]; then
    echo "请在 nav_grasp_real/project.env 中配置可执行的 CAN_SETUP_SCRIPT。" >&2
    exit 2
fi

source "$ROS_SETUP"
source "$driver_setup"

if ros2 node list 2>/dev/null | grep -qx '/arm/agx_arm_ctrl_single_node'; then
    echo "错误：AGX机械臂驱动仍在运行。请先在驱动终端按 Ctrl-C，再运行本脚本。" >&2
    exit 2
fi

"$CAN_SETUP_SCRIPT"
export PYTHONPATH="$driver_root/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
exec python3 "$workspace_root/tools/calibrate_gripper.py" "$@"
