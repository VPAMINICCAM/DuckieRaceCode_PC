#!/usr/bin/env python3
"""Static contracts shared by every PC-side Joy publisher."""

import ast
import unittest
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


if __name__ == "__main__":
    unittest.main()
