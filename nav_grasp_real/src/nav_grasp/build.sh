#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$workspace_root/../../.." && pwd)"
source "$repo_root/scripts/project_env.bash"

set +u
source "$ROS_SETUP"
set -u

cd "$workspace_root"
colcon build --symlink-install --event-handlers console_direct+
