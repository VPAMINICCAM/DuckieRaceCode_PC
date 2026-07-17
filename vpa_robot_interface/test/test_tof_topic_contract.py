#!/usr/bin/env python3
"""Static topic/namespace contracts for both ToF interfaces."""

import ast
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
DRIVERS = (
    PACKAGE / "scripts/tof_driver_i2c.py",
    PACKAGE / "scripts/tof_driver_usart.py",
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


if __name__ == "__main__":
    unittest.main()
