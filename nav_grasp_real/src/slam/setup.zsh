#!/usr/bin/env zsh

SLAM_WS_ROOT="${${(%):-%x}:A:h}"
NAV_GRASP_ROOT="${SLAM_WS_ROOT:h:h:h}"
project_env_file="${NAV_GRASP_ENV_FILE:-$NAV_GRASP_ROOT/nav_grasp_real/project.env}"
[[ -f "$project_env_file" ]] && source "$project_env_file"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/$ROS_DISTRO/setup.zsh}"

source "$ROS_SETUP"
for setup_file in "${BASE_DRIVER_SETUP:-}" "${LIDAR_DRIVER_SETUP:-}" \
        "${ROBOT_MODEL_SETUP:-}"; do
  [[ -n "$setup_file" && -f "$setup_file" ]] && source "$setup_file"
done
if [[ -f "$SLAM_WS_ROOT/install/setup.zsh" ]]; then
  source "$SLAM_WS_ROOT/install/setup.zsh"
fi

unset SLAM_WS_ROOT NAV_GRASP_ROOT project_env_file setup_file
