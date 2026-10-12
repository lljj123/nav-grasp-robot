#!/usr/bin/env bash
set -eo pipefail

workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "$workspace_root/.." && pwd)"
source "$repo_root/scripts/project_env.bash"

controller="${ARM_CONTROL_SCRIPT:-}"

if [[ -z "$controller" || ! -x "$controller" ]]; then
    echo "未配置可执行的 ARM_CONTROL_SCRIPT。" >&2
    echo "请复制 config/project.env.example 为 config/project.env 后填写。" >&2
    exit 2
fi

exec "$controller" "$@"
