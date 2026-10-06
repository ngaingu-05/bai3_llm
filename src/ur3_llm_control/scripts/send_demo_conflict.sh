#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
source ~/ur3_ws/install/setup.bash
source ~/ur3_ws/src/ur3_llm_control/scripts/source_llm_env.sh

echo "===================================================================="
echo "DEMO BÀI 03: XỬ LÝ XUNG ĐỘT ZONE (Zone B đang bị chiếm bởi blue_cube)"
echo "Command: Put the red cube in zone B"
echo "===================================================================="
ros2 topic pub --once /llm_task/command std_msgs/msg/String '{data: "Put the red cube in zone B"}'

