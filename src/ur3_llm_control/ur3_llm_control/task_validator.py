"""Validate LLM-generated skill plans before they reach a robot executor."""

from __future__ import annotations

import json
import re
from typing import Any


class PlanValidationError(ValueError):
    """Raised when a generated plan is not safe or supported."""


def _strip_json_markdown_fence(response: str) -> str:
    match = re.fullmatch(r"\s*```(?:json)?\s*\n?(.*?)\n?```\s*", response, re.IGNORECASE | re.DOTALL)
    return match.group(1).strip() if match else response.strip()


def validate_plan(
    response: str | dict[str, Any], objects: set[str], zones: set[str]
) -> list[dict[str, str]]:
    """Parse and validate a plan, returning canonical skill dictionaries."""
    try:
        payload = (
            json.loads(_strip_json_markdown_fence(response))
            if isinstance(response, str)
            else response
        )
    except json.JSONDecodeError as error:
        raise PlanValidationError("LLM response is not valid JSON") from error

    if not isinstance(payload, dict) or set(payload) != {"plan"}:
        raise PlanValidationError("Plan response must contain only a 'plan' field")

    raw_steps = payload["plan"]
    if not isinstance(raw_steps, list) or not raw_steps:
        raise PlanValidationError("Plan must be a non-empty list")

    plan: list[dict[str, str]] = []
    carrying: str | None = None
    completed_home = False

    for index, step in enumerate(raw_steps):
        if not isinstance(step, dict):
            raise PlanValidationError(f"Step {index + 1} must be an object")
        if "task" in step and "skill" not in step:
            step["skill"] = step.pop("task")
        if not isinstance(step.get("skill"), str):
            raise PlanValidationError(f"Step {index + 1} must have a 'skill' string")

        skill = step["skill"]
        if skill == "pick":
            if set(step) != {"skill", "object"}:
                raise PlanValidationError("pick requires exactly 'skill' and 'object'")
            object_id = step["object"]
            if object_id not in objects:
                raise PlanValidationError(f"Unknown object: {object_id}")
            if carrying is not None:
                raise PlanValidationError("Cannot pick another object before placing the current one")
            if plan and plan[-1]["skill"] == "home":
                plan.pop()
            completed_home = False
            carrying = object_id
            plan.append({"skill": "pick", "object": object_id})
        elif skill == "place":
            if set(step) != {"skill", "object", "zone"}:
                raise PlanValidationError("place requires exactly 'skill', 'object', and 'zone'")
            object_id = step["object"]
            zone_id = step["zone"]
            if object_id not in objects:
                raise PlanValidationError(f"Unknown object: {object_id}")
            if zone_id not in zones:
                raise PlanValidationError(f"Unknown zone: {zone_id}")
            if carrying != object_id:
                raise PlanValidationError("place must match the object from the preceding pick")
            carrying = None
            plan.append({"skill": "place", "object": object_id, "zone": zone_id})
        elif skill == "home":
            if set(step) != {"skill"}:
                raise PlanValidationError("home accepts no parameters")
            if carrying is not None:
                raise PlanValidationError("Cannot return home while holding an object")
            if not plan or plan[-1]["skill"] != "home":
                plan.append({"skill": "home"})
            completed_home = True
        else:
            raise PlanValidationError(f"Unsupported skill: {skill}")

    if carrying is not None:
        raise PlanValidationError("Plan ends before the picked object is placed")
    if not completed_home or plan[-1]["skill"] != "home":
        raise PlanValidationError("Plan must finish with home")
    if len(plan) > 1 and not any(step["skill"] == "pick" for step in plan):
        raise PlanValidationError("Plan must contain at least one pick/place pair")

    return plan