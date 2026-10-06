#!/usr/bin/env bash
set -eo pipefail

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
config_file="$workspace_root/src/ur3_llm_control/config/9router.env"

if [[ ! -f "$config_file" ]]; then
  printf 'Missing %s\n' "$config_file" >&2
  exit 1
fi

set -a
source "$config_file"
set +a

if [[ -z "${LLM_API_KEY:-}" || "$LLM_API_KEY" == "PASTE_YOUR_9ROUTER_API_KEY_HERE" ]]; then
  printf 'Set LLM_API_KEY in %s before starting.\n' "$config_file" >&2
  exit 1
fi

# Ensure 9Router server is running on port 20128
if ! ss -tlnp 2>/dev/null | grep -q ":20128 "; then
  echo "Starting 9Router server on port 20128..."
  PORT=20128 HOSTNAME=0.0.0.0 node /usr/lib/node_modules/9router/app/server.js > /dev/null 2>&1 &
  sleep 2
fi

# Clean up any lingering old simulation processes to avoid port/controller conflicts
pkill -9 -f "robot_skills" 2>/dev/null || true
pkill -9 -f "ros_gz_bridge" 2>/dev/null || true
pkill -9 -f "gz sim" 2>/dev/null || true
sleep 1

if [[ -f "/opt/ros/jazzy/setup.bash" ]]; then
  source /opt/ros/jazzy/setup.bash
elif [[ -f "/opt/ros/humble/setup.bash" ]]; then
  source /opt/ros/humble/setup.bash
fi

source "$workspace_root/install/setup.bash"
set -u
exec ros2 launch ur3_llm_robot_skills llm_robot.launch.py \
  ur_type:=ur3e gazebo_gui:=true launch_rviz:=true
