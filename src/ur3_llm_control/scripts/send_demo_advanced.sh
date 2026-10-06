#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null
source ~/ur3_ws/install/setup.bash
source ~/ur3_ws/src/ur3_llm_control/scripts/source_llm_env.sh

echo "Sending advanced command: Arrange all objects according to my student ID"
ros2 topic pub --once /llm_task/command std_msgs/msg/String '{data: "Arrange all objects according to my student ID"}'
