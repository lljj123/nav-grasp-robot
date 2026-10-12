#!/usr/bin/env bash
set -eo pipefail

workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
launcher="$workspace_root/src/nav_grasp_mission/scripts/start_full_mission.sh"
required_version='LAUNCHER_VERSION="2026-10-12-portable-v4"'

if [[ ! -x "$launcher" ]]; then
    echo "错误：受管启动器不存在或不可执行：$launcher" >&2
    exit 2
fi
if ! grep -Fq -- "$required_version" "$launcher"; then
    echo '错误：受管启动器版本校验失败，文件可能被旧版本覆盖。' >&2
    echo "请检查：$launcher" >&2
    exit 2
fi

exec "$launcher" "$@"
