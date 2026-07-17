#!/usr/bin/env python3
"""Hardware-free deployment contracts for the robot DuckieRace node."""

import ast
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
NODE_PATH = PACKAGE / "scripts/duckierace.py"
LAUNCH_PATH = PACKAGE / "launch/duckierace_start.launch"


def self_assignment_lines(function):
    assignments = {}
    for node in ast.walk(function):
        if not isinstance(node, (ast.Assign, ast.AnnAssign)):
            continue
        targets = node.targets if isinstance(node, ast.Assign) else [node.target]
        for target in targets:
            if (isinstance(target, ast.Attribute)
                    and isinstance(target.value, ast.Name)
                    and target.value.id == "self"):
                assignments.setdefault(target.attr, node.lineno)
    return assignments


def rospy_call_lines(function, method):
    lines = []
    for node in ast.walk(function):
        if not isinstance(node, ast.Call):
            continue
        called = node.func
        if (isinstance(called, ast.Attribute)
                and isinstance(called.value, ast.Name)
                and called.value.id == "rospy"
                and called.attr == method):
            lines.append(node.lineno)
    return lines


class DuckieRaceDeploymentContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = NODE_PATH.read_text(encoding="utf-8")
        cls.tree = ast.parse(cls.source, filename=str(NODE_PATH))
        line_follower = next(
            node for node in cls.tree.body
            if isinstance(node, ast.ClassDef) and node.name == "LineFollower")
        cls.init = next(
            node for node in line_follower.body
            if isinstance(node, ast.FunctionDef) and node.name == "__init__")

    def test_external_charging_authority_is_the_safe_default(self):
        self.assertIn(
            'rospy.get_param("~autonomous_mode", False)', self.source)

        launch = ET.parse(LAUNCH_PATH).getroot()
        autonomous_arg = next(
            item for item in launch.findall("arg")
            if item.get("name") == "autonomous_mode")
        self.assertEqual("false", autonomous_arg.get("default"))

        autonomous_param = next(
            item for item in launch.iter("param")
            if item.get("name") == "autonomous_mode")
        self.assertEqual("$(arg autonomous_mode)", autonomous_param.get("value"))

    def test_callback_state_and_publishers_precede_every_endpoint(self):
        assignments = self_assignment_lines(self.init)
        endpoint_lines = (
            rospy_call_lines(self.init, "Subscriber")
            + rospy_call_lines(self.init, "Timer")
        )
        self.assertTrue(endpoint_lines)
        first_endpoint = min(endpoint_lines)

        required_before_callbacks = (
            "power_level",
            "low_power_threshold",
            "in_fuel_zone",
            "charging",
            "real_spd",
            "last_joy_time",
            "autonomous_mode",
            "want_charge",
            "cmd_pub",
            "brake_pub",
            "power_pub",
            "spd_pub",
        )
        for attribute in required_before_callbacks:
            with self.subTest(attribute=attribute):
                self.assertIn(attribute, assignments)
                self.assertLess(assignments[attribute], first_endpoint)

    def test_robot_button_indices_match_the_pc_contract(self):
        assignments = self_assignment_lines(self.init)
        expected = {
            "button_x": 0,
            "button_b": 2,
            "button_y": 3,
            "button_l1": 4,
            "button_r1": 5,
        }
        for name, value in expected.items():
            assignment = next(
                node for node in ast.walk(self.init)
                if isinstance(node, ast.Assign)
                and any(
                    isinstance(target, ast.Attribute)
                    and target.attr == name
                    for target in node.targets
                )
            )
            self.assertIsInstance(assignment.value, ast.Constant)
            self.assertEqual(value, assignment.value.value)
            self.assertIn(name, assignments)

    def test_lost_line_branch_publishes_an_explicit_stop(self):
        line_follower = next(
            node for node in self.tree.body
            if isinstance(node, ast.ClassDef) and node.name == "LineFollower")
        image_callback = next(
            node for node in line_follower.body
            if isinstance(node, ast.FunctionDef)
            and node.name == "image_callback")

        for branch in (node for node in ast.walk(image_callback)
                       if isinstance(node, ast.If)):
            marks_line_lost = any(
                isinstance(node, ast.Assign)
                and any(
                    isinstance(target, ast.Attribute)
                    and target.attr == "lost_detect"
                    for target in node.targets
                )
                and isinstance(node.value, ast.Constant)
                and node.value.value is True
                for node in ast.walk(branch)
            )
            publishes_stop = any(
                isinstance(node, ast.Call)
                and isinstance(node.func, ast.Attribute)
                and node.func.attr == "publish_twist"
                and len(node.args) == 2
                and all(
                    isinstance(argument, ast.Constant)
                    and argument.value == 0.0
                    for argument in node.args
                )
                for node in ast.walk(branch)
            )
            if marks_line_lost and publishes_stop:
                return
        self.fail("Lost-line branch does not publish a zero Twist")

    def test_tof_include_has_no_nested_robot_namespace(self):
        launch = ET.parse(LAUNCH_PATH).getroot()
        use_tof_arg = next(
            item for item in launch.findall("arg")
            if item.get("name") == "use_tof")
        self.assertEqual(
            "$(eval arg('robot_name') == 'daisy')",
            use_tof_arg.get("default"),
        )
        nested_robot_groups = [
            group for group in launch.iter("group")
            if group.get("ns") == "robot"
        ]
        self.assertEqual([], nested_robot_groups)


if __name__ == "__main__":
    unittest.main()
