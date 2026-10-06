# UR3 LLM Skill Planner

ROS 2 demo for natural-language UR3/UR3e pick-and-place. The LLM selects only the fixed `pick`, `place`, and `home` skills. It never supplies coordinates, joint values, or trajectories. A Python validator checks the plan before publishing it, and the C++ executor checks it again before using MoveIt.

## Current environment

This workspace currently has ROS 2 Jazzy and Gazebo Harmonic installed. The supplied assignment specifies Ubuntu 22.04 and ROS 2 Humble, so this implementation has been built and smoke-tested on Jazzy, not certified on Humble. Use a Humble workspace if the course strictly requires that distro.

## Student configuration

`ur3_llm_control/config/student_config.yaml` is configured as:

```yaml
student_name: Nghia
student_id: "23020755"
```

The planner calculates `55 % 6 = 1` and checks the expected assignment before starting:

| Zone | Assigned cube |
| --- | --- |
| A | Red |
| B | Blue |
| C | Yellow |

Edit this YAML with the exact name and ID to use in the submitted demo. `zone_assignments` must agree with the final two digits of `student_id`.

## Architecture

```text
/llm_task/command
  -> llm_planner (9Router OpenAI-compatible chat endpoint)
  -> task_validator (skill, object, zone, and sequence whitelist)
  -> /llm_task/validated_plan
  -> robot_skills (independent second validator)
  -> MoveIt 2 + parallel-gripper trajectory controller
  -> Gazebo UR3/UR3e
```

The Gazebo world includes the robot, table, and three colored cubes. A MoveIt planning-scene table and cube collision objects are populated from `ur3_llm_robot_skills/config/scene.yaml`. The planner rejects unknown skills/IDs, extra fields, mismatched pick/place pairs, unfinished plans, and plans that do not end with `home`.

The current table center is `(-0.30, 0, 0.225)` m with size `0.14 x 0.80 x 0.05` m. Cube centers are at `z=0.27` m and `x=-0.30` m, with `y` positions `-0.12`, `0.03`, and `0.18` m. Zone centers are at `z=0.27` m and `x=-0.25` m, with `y` positions `-0.30`, `-0.12`, and `0.28` m to keep destinations clear of the other source cubes. The table, cubes, and zones were lowered 0.10 m together so the cubes remain on the tabletop. The rows were shifted 0.10 m along negative y to clear the parked gripper, and the tabletop was shortened toward the robot to preserve arm clearance. Verify the TCP remains stable in the active simulator before motion; observed idle joint drift makes measured poses provisional.

## Build

Install ROS 2, MoveIt 2, Gazebo Sim, and the Universal Robots ROS 2 simulation packages for the selected ROS distribution. From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-up-to ur3_llm_control ur3_llm_robot_skills --symlink-install
source install/setup.bash
```

## Configure 9Router

Create/select a model in 9Router and obtain its OpenAI-compatible chat-completions endpoint, model ID, and API key from your 9Router account. In the shell that will run the launch file, set:

```bash
export LLM_ENDPOINT="https://YOUR_9ROUTER_ENDPOINT/v1/chat/completions"
export LLM_MODEL="YOUR_ROUTER_MODEL_ID"
read -rsp "9Router API key: " LLM_API_KEY; echo
export LLM_API_KEY
```

Keep the key in the shell or a private local environment file; do not commit it. The node sends JSON-mode chat-completions requests and expects the model response in `choices[0].message.content`.

For a one-file setup, edit `src/ur3_llm_control/config/9router.env` once, put the API key in `LLM_API_KEY`, then run:

```bash
src/ur3_llm_control/scripts/run_llm_robot.sh
```

That script exports the endpoint, model, and key before launching Gazebo, MoveIt, the planner, and the skill executor. The credential file is ignored by git.

For another terminal, load the same environment without launching a second simulator:

```bash
source src/ur3_llm_control/scripts/source_llm_env.sh
ros2 topic echo /llm_task/execution_status
```

Then send commands from that terminal with `ros2 topic pub`.

## Run the demo

Start Gazebo, MoveIt, the planner, and skill executor:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch ur3_llm_robot_skills llm_robot.launch.py ur_type:=ur3e
```

Use `ur_type:=ur3` for UR3. Once the Gazebo/MoveIt startup delay has passed, publish a natural-language request from another sourced terminal:

```bash
ros2 topic pub --once /llm_task/command std_msgs/msg/String \
  '{data: "Move the red cube to zone B"}'
```

Read execution results on `/llm_task/execution_status` or in the executor terminal:

```bash
ros2 topic echo /llm_task/execution_status
```

For the advanced task, try `Arrange all objects according to my student ID.` The mapping is supplied to the LLM in its system prompt; each object is picked and placed sequentially before the plan ends with `home()`.

## Notes and limitations

- MoveIt plans with collision checking against the configured table and cubes. If a target cannot be planned or executed, the current skill reports `FAILED` and later skills are not run.
- The gripper uses one actuated sliding finger opposite a fixed finger and `gripper_controller/follow_joint_trajectory`. Check that the cube remains between the fingers in Gazebo when preparing the final video; physical grasp/contact behavior depends on the local Gazebo physics version.
- The world starts with cubes at their configured source positions. The current assignment is a basic sorter and does not clear already occupied target zones or recover a dropped object.
- The source UR packages are upstream workspace dependencies. The custom gripper, world, scene, and launch are kept in `ur3_llm_robot_skills`.
- Unit tests cover valid single/multi-object plans and invalid skills, IDs, pair ordering, and missing `home`. The ROS package build, Xacro, SRDF, and SDF have been checked on the current Jazzy machine.
- The current headless smoke test starts Gazebo, MoveIt, the arm controller, gripper controller, planner, and skill executor. Earlier scene coordinates caused MoveIt to reject the initial state due to arm/table collision. Scene coordinates have since been moved and their parameter overrides verified in the running executor, but a successful pick/place test with the updated coordinates has not yet been confirmed. Keep MoveIt's collision checking enabled.
- The external 9Router request and a full successful pick/place run still need validation with the student's account and demo setup.