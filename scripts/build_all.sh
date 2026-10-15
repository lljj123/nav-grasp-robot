#!/usr/bin/env bash
set -euo pipefail

NAV_GRASP_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$NAV_GRASP_ROOT/scripts/project_env.bash"

"$SLAM_WS/build.sh"
"$VISUAL_WS/build.sh"
"$MISSION_WS/build.sh"

echo "All nav-grasp real workspaces built successfully."
