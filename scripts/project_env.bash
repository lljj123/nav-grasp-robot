#!/usr/bin/env bash
# Shared paths for repository scripts. This file may be sourced from Bash.

if [[ -z "${NAV_GRASP_ROOT:-}" ]]; then
    NAV_GRASP_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
fi

project_env_file="${NAV_GRASP_ENV_FILE:-$NAV_GRASP_ROOT/config/project.env}"
if [[ -f "$project_env_file" ]]; then
    # shellcheck disable=SC1090
    source "$project_env_file"
fi
unset project_env_file

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/$ROS_DISTRO/setup.bash}"
SLAM_WS="${SLAM_WS:-$NAV_GRASP_ROOT/slam_ws}"
VISUAL_WS="${VISUAL_WS:-$NAV_GRASP_ROOT/visual_grasp_ws}"
MISSION_WS="${MISSION_WS:-$NAV_GRASP_ROOT/nav_grasp_ws}"

export NAV_GRASP_ROOT ROS_DISTRO ROS_SETUP SLAM_WS VISUAL_WS MISSION_WS

nav_grasp_source_if_present() {
    local setup_file="${1:-}"
    if [[ -n "$setup_file" && -f "$setup_file" ]]; then
        # shellcheck disable=SC1090
        source "$setup_file"
    fi
}

