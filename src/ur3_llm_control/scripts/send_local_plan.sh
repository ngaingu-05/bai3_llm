#!/usr/bin/env bash
set -euo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
source "$workspace_root/install/setup.bash"

plan='{
  "plan": [
    {"skill": "pick", "object": "blue_cube"},
    {"skill": "place", "object": "blue_cube", "zone": "temp_1"},
    {"skill": "pick", "object": "red_cube"},
    {"skill": "place", "object": "red_cube", "zone": "zone_b"},
    {"skill": "home"}
  ]
}'

printf '%s\n' 'Sending local plan without LLM:'
printf '%s\n' "$plan"
ros2 topic pub --once /llm_task/validated_plan std_msgs/msg/String "data: '$plan'"

printf '%s\n' 'Waiting for local execution result...'
set +e
set +o pipefail
timeout 180 ros2 topic echo --no-arr /llm_task/execution_status \
  | awk '/TASK SUCCESS|TASK FAILED/ { print; if ($0 ~ /TASK SUCCESS/) exit 0; exit 1 }'
result=$?
set -e

if [[ $result -eq 0 ]]; then
  printf '%s\n' 'Local plan completed successfully.'
else
  printf '%s\n' 'Local plan failed or timed out.' >&2
  exit "$result"
fi
