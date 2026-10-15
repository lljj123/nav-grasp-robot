#!/usr/bin/env bash
set -eo pipefail

workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$workspace_root/../../.." && pwd)"
source "$repo_root/scripts/project_env.bash"
driver_root="${ARM_DRIVER_WS:-}"
driver_setup="${ARM_DRIVER_SETUP:-${driver_root:+$driver_root/install/setup.bash}}"
driver_executable="$driver_root/install/agx_arm_ctrl/lib/agx_arm_ctrl/agx_arm_ctrl_single"

if [[ -z "$driver_root" || ! -f "$driver_setup" ]]; then
    echo "请在 nav_grasp_real/project.env 中配置 ARM_DRIVER_WS/ARM_DRIVER_SETUP。" >&2
    exit 2
fi
if [[ -z "${CAN_SETUP_SCRIPT:-}" || ! -x "$CAN_SETUP_SCRIPT" ]]; then
    echo "请在 nav_grasp_real/project.env 中配置可执行的 CAN_SETUP_SCRIPT。" >&2
    exit 2
fi

if pgrep -f "^/usr/bin/python3 ${driver_executable}( |$)" >/dev/null; then
    echo "错误：机械臂控制驱动仍在运行。请先在驱动终端按 Ctrl-C，再测量夹爪手动行程。" >&2
    exit 2
fi

"$CAN_SETUP_SCRIPT"
source "$ROS_SETUP"
source "$driver_setup"

python_version="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
pyagxarm_site="$driver_root/.venv/lib/python${python_version}/site-packages"
if [[ ! -d "$pyagxarm_site/pyAgxArm" ]]; then
    echo "错误：未找到 pyAgxArm：$pyagxarm_site/pyAgxArm" >&2
    exit 2
fi
export PYTHONPATH="$pyagxarm_site:${PYTHONPATH:-}"

tool_args=()
if [[ -n "${NAV_GRASP_GRIPPER_REPORT:-}" ]]; then
    tool_args+=(--output "$NAV_GRASP_GRIPPER_REPORT")
fi
exec python3 "$workspace_root/tools/measure_gripper_range.py" \
    "${tool_args[@]}" "$@"
