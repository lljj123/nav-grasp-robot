#!/usr/bin/env bash
set -eo pipefail

workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$workspace_root/../../.." && pwd)"
source "$repo_root/scripts/project_env.bash"
source "$ROS_SETUP"
nav_grasp_source_if_present "${ARM_DRIVER_SETUP:-}"
source "$workspace_root/install/setup.bash"

echo "正在确认机械臂已退出手动示教并进入 CAN 控制模式..."
if ! mode_result="$(timeout 8s ros2 service call \
    /arm/enter_can_control std_srvs/srv/Trigger '{}')"; then
    echo "CAN 模式恢复服务不可用或超时；请重启已更新的机械臂驱动。" >&2
    exit 1
fi
printf '%s\n' "$mode_result"

if [[ "$mode_result" != *"success=True"* && \
      "$mode_result" != *"success: true"* ]]; then
    echo "机械臂仍处于手动示教/非 CAN 模式，未生成规划也未发送运动命令。" >&2
    exit 1
fi

echo "正在生成最新抓取规划..."
plan_result="$(ros2 service call \
    /visual_grasp/plan_pick std_srvs/srv/Trigger '{}')"
printf '%s\n' "$plan_result"

if [[ "$plan_result" != *"success=True"* && \
      "$plan_result" != *"success: true"* ]]; then
    echo "规划失败，未启动机械臂运动。" >&2
    exit 1
fi

# Allow the freshly published pose sequence to reach the executor before the
# execution request.  The plan is still well inside its configured lifetime.
sleep 0.2
echo "规划成功，开始完整抓取..."
exec ros2 service call \
    /visual_grasp/execute_full_pick std_srvs/srv/Trigger '{}'
