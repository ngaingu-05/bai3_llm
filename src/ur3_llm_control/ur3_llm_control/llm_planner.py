"""ROS 2 node that converts natural-language commands into validated skills."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Optional
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import String

from ur3_llm_control.task_validator import PlanValidationError, validate_plan


class LlmPlanner(Node):
    def __init__(self) -> None:
        super().__init__("ur3_llm_planner")
        default_config = str(
            Path(get_package_share_directory("ur3_llm_control"))
            / "config"
            / "student_config.yaml"
        )
        self.declare_parameter("student_config", default_config)
        self.declare_parameter("command_topic", "/llm_task/command")
        self.declare_parameter("plan_topic", "/llm_task/validated_plan")
        self.declare_parameter("request_timeout", 60.0)
        self.config = self._load_config(self.get_parameter("student_config").value)
        self.objects = set(self.config["objects"])
        self.zones = set(self.config["zones"])
        self.temporary_positions = set(self.config["temporary_positions"])
        self.destinations = self.zones | self.temporary_positions

        # Latest scene state reported by camera detector
        self.current_scene: Optional[dict[str, Any]] = None

        self.plan_publisher = self.create_publisher(
            String, self.get_parameter("plan_topic").value, 10
        )
        self.command_subscription = self.create_subscription(
            String,
            self.get_parameter("command_topic").value,
            self._handle_command,
            10,
        )
        self.scene_subscription = self.create_subscription(
            String,
            "/scene_state",
            self._handle_scene_state,
            10,
        )
        self.get_logger().info("LLM planner ready; waiting for camera state and task commands.")

    def _load_config(self, config_path: str) -> dict:
        with open(config_path, encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file)
        student_id = str(config["student_id"])
        if not student_id.isdigit() or len(student_id) < 2:
            raise ValueError("student_id must contain at least two digits")
        permutation = int(student_id[-2:]) % 6
        config["student_permutation"] = permutation
        expected_zones = self._zone_assignments(permutation)
        if config["zone_assignments"] != expected_zones:
            raise ValueError("zone_assignments do not match the configured student_id")
        if not set(expected_zones).issubset(set(config["zones"])):
            raise ValueError("zones must define zone_a, zone_b, and zone_c")
        return config

    @staticmethod
    def _zone_assignments(permutation: int) -> dict[str, str]:
        arrangements = (
            ("red_cube", "yellow_cube", "blue_cube"),
            ("red_cube", "blue_cube", "yellow_cube"),
            ("yellow_cube", "red_cube", "blue_cube"),
            ("yellow_cube", "blue_cube", "red_cube"),
            ("blue_cube", "red_cube", "yellow_cube"),
            ("blue_cube", "yellow_cube", "red_cube"),
        )
        return dict(zip(("zone_a", "zone_b", "zone_c"), arrangements[permutation]))

    def _handle_scene_state(self, message: String) -> None:
        try:
            self.current_scene = json.loads(message.data)
        except Exception:
            pass

    def _format_scene_prompt(self) -> str:
        """Format the current environment state detected by the camera."""
        if not self.current_scene or "zones" not in self.current_scene:
            raise RuntimeError("No camera scene state received; wait for /scene_state before sending a task")

        zones_state = self.current_scene["zones"]
        objects_state = self.current_scene.get("objects", {})
        temporary_state = self.current_scene.get("temporary_positions", {})

        lines = ["Current Camera Scene State:"]
        lines.append("- Zone Occupancy:")
        for z in sorted(zones_state.keys()):
            occ = zones_state.get(z, "empty")
            lines.append(f"  * {z}: {occ}")
        lines.append("- Object Locations:")
        for obj, info in sorted(objects_state.items()):
            pos = info.get("position", "table") if isinstance(info, dict) else info
            lines.append(f"  * {obj}: {pos}")
        lines.append("- Temporary-position occupancy (not task zones):")
        for position in sorted(temporary_state):
            lines.append(f"  * {position}: {temporary_state[position]}")
        return "\n".join(lines)

    def _system_prompt(self) -> str:
        assignment = ", ".join(
            f"{zone}={object_id}" for zone, object_id in self.config["zone_assignments"].items()
        )
        scene_info = self._format_scene_prompt()

        return (
            "You are a task planner for a UR3 robot with an overhead camera. "
            "Understand the user's request and output a sequence of robot skills. "
            "Never output joint values, trajectories, poses, coordinates, or controller commands.\n\n"
            f"{scene_info}\n\n"
            "AVAILABLE SKILLS:\n"
            "- pick(object): grasps an object from its current location\n"
            "- place(object, destination): places the grasped object at a task zone or an empty temporary position\n"
            "- home(): returns arm to safe home position\n\n"
            "CRITICAL CONFLICT RESOLUTION RULES:\n"
            "1. Each zone can only hold ONE object. You MUST NEVER place an object into an occupied zone.\n"
            "2. If the user wants to place object X into zone Z, but zone Z is currently occupied by object Y:\n"
            "   You MUST FIRST clear zone Z by moving object Y to an available temporary position ('temp_1' or 'temp_2'):\n"
            "     a. pick(Y)\n"
            "     b. place(Y, temp_1)\n"
            "     c. pick(X)\n"
            "     d. place(X, Z)\n"
            "     e. home()\n"
            "3. If zone Z is already empty, directly execute: pick(X), place(X, Z), home().\n"
            "4. For student ID arrangement commands ('Arrange all objects according to my student ID'):\n"
            f"   Target configuration: {assignment}.\n"
            "   Carefully check which zones are occupied. Move any blocking cubes to temp positions first if needed, "
            "then place the required cubes into their assigned zones in sequence, and finish with home().\n\n"
            "OUTPUT FORMAT:\n"
            "Return EXACTLY ONE JSON object with a 'plan' array and NO markdown formatting.\n"
            'Schema: {"plan":[{"skill":"pick","object":"..."},{"skill":"place","object":"...","zone":"..."},{"skill":"home"}]}.\n'
            f"Valid objects: {', '.join(sorted(self.objects))}.\n"
            f"Valid zones/destinations: {', '.join(sorted(self.destinations))}."
        )

    def _request_plan(self, command: str) -> str:
        endpoint = os.environ.get("LLM_ENDPOINT", "").strip()
        api_key = os.environ.get("LLM_API_KEY", "").strip()
        model = os.environ.get("LLM_MODEL", "").strip()
        if not endpoint or not api_key or not model:
            raise RuntimeError("Set LLM_ENDPOINT, LLM_API_KEY, and LLM_MODEL before sending commands")

        system_msg = self._system_prompt()
        self.get_logger().info(f"Generated System Prompt with Scene State:\n{system_msg}")

        request_body = json.dumps(
            {
                "model": model,
                "messages": [
                    {"role": "system", "content": system_msg},
                    {"role": "user", "content": command},
                ],
                "stream": False,
                "response_format": {"type": "json_object"},
                "temperature": 0,
            }
        ).encode("utf-8")
        request = Request(
            endpoint,
            data=request_body,
            headers={
                "Authorization": f"Bearer {api_key}",
                "Content-Type": "application/json",
            },
            method="POST",
        )
        try:
            with urlopen(
                request, timeout=float(self.get_parameter("request_timeout").value)
            ) as response:
                result = json.loads(response.read().decode("utf-8"))
        except HTTPError as error:
            raise RuntimeError(f"LLM endpoint returned HTTP {error.code}") from error
        except URLError as error:
            raise RuntimeError(f"Could not reach LLM endpoint: {error.reason}") from error

        try:
            content = result["choices"][0]["message"]["content"]
        except (KeyError, IndexError, TypeError) as error:
            raise RuntimeError("LLM response did not contain a chat completion") from error
        if not isinstance(content, str):
            raise RuntimeError("LLM response content must be a JSON string")
        return content

    def _handle_command(self, message: String) -> None:
        command = message.data.strip()
        if not command:
            self.get_logger().error("Empty user command; nothing was published.")
            return

        self.get_logger().info(f"USER COMMAND: {command}")
        try:
            raw_plan = self._request_plan(command)
            plan = validate_plan(raw_plan, self.objects, self.destinations)
        except (PlanValidationError, RuntimeError, json.JSONDecodeError) as error:
            self.get_logger().error(f"Plan rejected; robot was not commanded: {error}")
            return

        self.get_logger().info("LLM PLAN:")
        for index, step in enumerate(plan, start=1):
            arguments = ", ".join(value for key, value in step.items() if key != "skill")
            self.get_logger().info(f"{index}. {step['skill']}({arguments})")
        output = String()
        output.data = json.dumps({"plan": plan}, separators=(",", ":"))
        self.plan_publisher.publish(output)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LlmPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
