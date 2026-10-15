#!/usr/bin/env bash
set -euo pipefail

SLAM_WS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
NAV_GRASP_ROOT="$(cd -- "$SLAM_WS_ROOT/../../.." && pwd)"
source "$NAV_GRASP_ROOT/scripts/project_env.bash"

set +u
source "$ROS_SETUP"
nav_grasp_source_if_present "${BASE_DRIVER_SETUP:-}"
nav_grasp_source_if_present "${LIDAR_DRIVER_SETUP:-}"
nav_grasp_source_if_present "${ROBOT_MODEL_SETUP:-}"
set -u

cd "$SLAM_WS_ROOT"
colcon build --symlink-install
