import unittest

from ur3_llm_control.task_validator import PlanValidationError, validate_plan


OBJECTS = {"red_cube", "yellow_cube", "blue_cube", "green_cube", "purple_cube"}
ZONES = {"zone_a", "zone_b", "zone_c", "temp_1", "temp_2"}


class TestValidatePlan(unittest.TestCase):
    def test_accepts_single_object_plan(self):
        plan = {
            "plan": [
                {"skill": "pick", "object": "red_cube"},
                {"skill": "place", "object": "red_cube", "zone": "zone_b"},
                {"skill": "home"},
            ]
        }

        self.assertEqual(validate_plan(plan, OBJECTS, ZONES), plan["plan"])

    def test_accepts_home_only_plan(self):
        plan = {"plan": [{"skill": "home"}]}

        self.assertEqual(validate_plan(plan, OBJECTS, ZONES), plan["plan"])

    def test_accepts_conflict_resolution_plan(self):
        plan = {
            "plan": [
                {"skill": "pick", "object": "blue_cube"},
                {"skill": "place", "object": "blue_cube", "zone": "temp_1"},
                {"skill": "pick", "object": "red_cube"},
                {"skill": "place", "object": "red_cube", "zone": "zone_b"},
                {"skill": "home"},
            ]
        }

        self.assertEqual(validate_plan(plan, OBJECTS, ZONES), plan["plan"])

    def test_accepts_multi_object_plan(self):
        plan = {
            "plan": [
                {"skill": "pick", "object": "blue_cube"},
                {"skill": "place", "object": "blue_cube", "zone": "zone_a"},
                {"skill": "pick", "object": "red_cube"},
                {"skill": "place", "object": "red_cube", "zone": "zone_c"},
                {"skill": "home"},
            ]
        }

        self.assertEqual(validate_plan(plan, OBJECTS, ZONES), plan["plan"])

    def test_accepts_json_markdown_fence(self):
        response = '''```json
{"plan":[{"skill":"pick","object":"red_cube"},
{"skill":"place","object":"red_cube","zone":"zone_b"},
{"skill":"home"}]}
```'''

        self.assertEqual(
            validate_plan(response, OBJECTS, ZONES),
            [
                {"skill": "pick", "object": "red_cube"},
                {"skill": "place", "object": "red_cube", "zone": "zone_b"},
                {"skill": "home"},
            ],
        )

    def test_accepts_plain_json_string(self):
        response = (
            '{"plan":[{"skill":"pick","object":"red_cube"},'
            '{"skill":"place","object":"red_cube","zone":"zone_b"},'
            '{"skill":"home"}]}'
        )

        self.assertEqual(len(validate_plan(response, OBJECTS, ZONES)), 3)

    def test_rejects_unknown_skill(self):
        with self.assertRaisesRegex(PlanValidationError, "Unsupported skill"):
            validate_plan(
                {"plan": [{"skill": "move_joints"}]}, OBJECTS, ZONES
            )

    def test_rejects_unknown_object_and_zone(self):
        bad_object = {
            "plan": [
                {"skill": "pick", "object": "black_cube"},
                {"skill": "place", "object": "black_cube", "zone": "zone_a"},
                {"skill": "home"},
            ]
        }
        bad_zone = {
            "plan": [
                {"skill": "pick", "object": "red_cube"},
                {"skill": "place", "object": "red_cube", "zone": "zone_d"},
                {"skill": "home"},
            ]
        }

        with self.assertRaisesRegex(PlanValidationError, "Unknown object"):
            validate_plan(bad_object, OBJECTS, ZONES)
        with self.assertRaisesRegex(PlanValidationError, "Unknown zone"):
            validate_plan(bad_zone, OBJECTS, ZONES)

    def test_rejects_unpaired_or_incomplete_steps(self):
        mismatched = {
            "plan": [
                {"skill": "pick", "object": "red_cube"},
                {"skill": "place", "object": "blue_cube", "zone": "zone_a"},
                {"skill": "home"},
            ]
        }
        no_home = {
            "plan": [
                {"skill": "pick", "object": "red_cube"},
                {"skill": "place", "object": "red_cube", "zone": "zone_a"},
            ]
        }

        with self.assertRaisesRegex(PlanValidationError, "must match"):
            validate_plan(mismatched, OBJECTS, ZONES)
        with self.assertRaisesRegex(PlanValidationError, "finish with home"):
            validate_plan(no_home, OBJECTS, ZONES)


if __name__ == "__main__":
    unittest.main()