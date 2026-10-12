#!/usr/bin/env bash

if [[ -z "${BASH_VERSION:-}" ]]; then
  echo "当前不是 Bash；zsh 请使用当前工作区的 setup.zsh" >&2
  return 2
fi

SLAM_WS_NOUNSET=0
if [[ "$-" == *u* ]]; then
  SLAM_WS_NOUNSET=1
  set +u
fi

SLAM_WS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
NAV_GRASP_ROOT="$(cd -- "$SLAM_WS_ROOT/.." && pwd)"
source "$NAV_GRASP_ROOT/scripts/project_env.bash"

source "$ROS_SETUP"
nav_grasp_source_if_present "${BASE_DRIVER_SETUP:-}"
nav_grasp_source_if_present "${LIDAR_DRIVER_SETUP:-}"
nav_grasp_source_if_present "${ROBOT_MODEL_SETUP:-}"
if [[ -f "$SLAM_WS_ROOT/install/setup.bash" ]]; then
  source "$SLAM_WS_ROOT/install/setup.bash"
fi

unset SLAM_WS_ROOT
if [[ "$SLAM_WS_NOUNSET" -eq 1 ]]; then
  set -u
fi
unset SLAM_WS_NOUNSET
