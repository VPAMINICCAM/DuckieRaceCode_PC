#!/usr/bin/env python3
"""Static contracts shared by every PC-side Joy publisher."""

import ast
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


SRC_ROOT = Path(__file__).resolve().parents[2]
EXPECTED_BUTTONS = {
    "BTN_X": 0,
    "BTN_B": 2,
    "BTN_Y": 3,
    "BTN_L1": 4,
    "BTN_R1": 5,
}
CONTROLLERS = (
    SRC_ROOT / "vpa_duckierace/scripts/virtual_driver_node.py",
    SRC_ROOT / "vpa_duckierace/scripts/keyboard_joy_console.py",
    SRC_ROOT / "vpa_duckierace/q_table/driver_node.py",
)
VIRTUAL_DRIVER_LAUNCH = SRC_ROOT / "vpa_duckierace/launch/virtual_driver.launch"
ANTICOLLISION_PARAMS = (
    "anticollision_enabled",
    "anticollision_safety_rate",
    "front_range_stale_sec",
    "conservative_safe_distance",
    "conservative_emergency_distance",
    "conservative_resume_distance",
    "conservative_ttc_slow",
    "conservative_ttc_brake",
    "aggressive_safe_distance",
    "aggressive_emergency_distance",
    "aggressive_resume_distance",
    "aggressive_ttc_slow",
    "aggressive_ttc_brake",
)


def module_integer_assignments(path):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    values = {}
    for statement in tree.body:
        if not isinstance(statement, ast.Assign) or len(statement.targets) != 1:
            continue
        target = statement.targets[0]
        if isinstance(target, ast.Name) and isinstance(statement.value, ast.Constant):
            if isinstance(statement.value.value, int):
                values[target.id] = statement.value.value
    return values


class JoyButtonContractTests(unittest.TestCase):
    def test_every_pc_controller_matches_the_physical_robot(self):
        for path in CONTROLLERS:
            with self.subTest(controller=str(path)):
                assignments = module_integer_assignments(path)
                actual = {
                    name: assignments.get(name) for name in EXPECTED_BUTTONS
                }
                self.assertEqual(EXPECTED_BUTTONS, actual)

    def test_anticollision_profiles_are_wired_to_both_virtual_drivers(self):
        launch = ET.parse(VIRTUAL_DRIVER_LAUNCH).getroot()
        launch_args = {item.get("name") for item in launch.findall("arg")}
        self.assertTrue(set(ANTICOLLISION_PARAMS).issubset(launch_args))

        driver_nodes = [
            node for node in launch.findall("node")
            if node.get("type") == "virtual_driver_node.py"
        ]
        self.assertEqual(2, len(driver_nodes))
        for node in driver_nodes:
            with self.subTest(node=node.get("name")):
                params = {
                    item.get("name"): item.get("value")
                    for item in node.findall("param")
                }
                for name in ANTICOLLISION_PARAMS:
                    self.assertEqual("$(arg {})".format(name), params.get(name))
                robot = "lucas" if "lucas" in node.get("name") else "daisy"
                self.assertEqual(
                    "$(arg {}_anticollision_sensor_required)".format(robot),
                    params.get("anticollision_sensor_required"),
                )


if __name__ == "__main__":
    unittest.main()
