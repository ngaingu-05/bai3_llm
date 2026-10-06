#!/usr/bin/env bash

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
config_file="$workspace_root/src/ur3_llm_control/config/9router.env"

if [[ ! -f "$config_file" ]]; then
  printf 'Missing %s\n' "$config_file" >&2
  return 1 2>/dev/null || exit 1
fi

source /opt/ros/jazzy/setup.bash
source "$workspace_root/install/setup.bash"

set -a
source "$config_file"
set +a

if [[ -z "${LLM_API_KEY:-}" || "$LLM_API_KEY" == "PASTE_YOUR_9ROUTER_API_KEY_HERE" ]]; then
  printf 'Set LLM_API_KEY in %s first.\n' "$config_file" >&2
  return 1 2>/dev/null || exit 1
fi

printf 'ROS 2 workspace and 9Router environment loaded.\n'
