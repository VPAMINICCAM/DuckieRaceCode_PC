#!/usr/bin/env python3
"""Static sensor and motor-boundary safety contracts."""

import ast
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
DRIVERS = (
    PACKAGE / "scripts/tof_driver_i2c.py",
    PACKAGE / "scripts/tof_driver_usart.py",
)
WHEEL_DRIVERS = (
    PACKAGE / "scripts/wheel_driver.py",
    PACKAGE / "scripts/wheel_driver_enhanced.py",
)


def publisher_topics(path):
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    topics = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not node.args:
            continue
        called = node.func
        if not (isinstance(called, ast.Attribute)
                and called.attr == "Publisher"):
            continue
        topic = node.args[0]
        if isinstance(topic, ast.Constant) and isinstance(topic.value, str):
            topics.append(topic.value)
    return topics


class ToFTopicContractTests(unittest.TestCase):
    def test_both_drivers_publish_the_robot_relative_contract(self):
        for driver in DRIVERS:
            with self.subTest(driver=str(driver)):
                source = driver.read_text(encoding="utf-8")
                topics = publisher_topics(driver)
                self.assertIn("front_range", topics)
                self.assertIn("front_range_status", topics)
                self.assertTrue(all(not topic.startswith("/") for topic in topics))
                self.assertIn("float('inf')", source)

    def test_launch_does_not_add_a_second_robot_namespace(self):
        launch = ET.parse(PACKAGE / "launch/vpa_tof.launch").getroot()
        self.assertFalse(any(
            element.get("ns") == "robot" for element in launch.iter()
        ))

    def test_collision_owner_is_enforced_at_the_motor_boundary(self):
        for driver in WHEEL_DRIVERS:
            with self.subTest(driver=str(driver)):
                source = driver.read_text(encoding="utf-8")
                self.assertIn('"collision_brake_cmd", Bool', source)
                self.assertIn('"driver_brake_active", Bool', source)
                self.assertIn("self.collision_estop", source)
                self.assertIn("self.driver_estop", source)
                self.assertIn("threading.RLock()", source)
                self.assertIn("self._stop_motors_locked()", source)
        base = WHEEL_DRIVERS[0].read_text(encoding="utf-8")
        enhanced = WHEEL_DRIVERS[1].read_text(encoding="utf-8")
        self.assertIn(
            "not self.driver_estop and not self.collision_estop", base)
        self.assertIn(
            "or self.driver_estop or self.collision_estop", enhanced)
        base_tree = ast.parse(base, filename=str(WHEEL_DRIVERS[0]))
        base_class = next(
            node for node in base_tree.body
            if isinstance(node, ast.ClassDef) and node.name == "WheelDriverNode"
        )
        base_methods = {
            node.name for node in base_class.body
            if isinstance(node, ast.FunctionDef)
        }
        self.assertTrue({
            "_stop_motors_locked",
            "_car_cmd_cb_locked",
            "_wheel_omega_cb_locked",
        }.issubset(base_methods))
        self.assertIn("self.omega_left_ref = 0", base)
        self.assertIn("self.omega_right_ref = 0", base)
        enhanced_tree = ast.parse(enhanced, filename=str(WHEEL_DRIVERS[1]))
        enhanced_class = next(
            node for node in enhanced_tree.body
            if isinstance(node, ast.ClassDef)
            and node.name == "WheelDriverEnhanced"
        )
        methods = {
            node.name for node in enhanced_class.body
            if isinstance(node, ast.FunctionDef)
        }
        self.assertTrue({
            "estop_driver_cb",
            "estop_collision_cb",
            "_stop_motors_locked",
            "_car_cmd_cb_locked",
        }.issubset(methods))


if __name__ == "__main__":
    unittest.main()
