#!/usr/bin/env python3
"""Focused, hardware-free checks for the virtual-driver charging policy."""

import importlib.util
import sys
import types
import unittest
from pathlib import Path


class _Clock:
    now = 0.0


class _Stamp:
    def to_sec(self):
        return _Clock.now


class _Header:
    stamp = None


class _Message:
    def __init__(self, data=None, **kwargs):
        self.data = data
        self.__dict__.update(kwargs)


class _Joy:
    def __init__(self):
        self.header = _Header()
        self.buttons = []
        self.axes = []


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _load_driver_module():
    """Load the node with only the tiny ROS/message surface the tests need."""
    rospy = types.ModuleType("rospy")
    rospy.Time = type("Time", (), {"now": staticmethod(lambda: _Stamp())})
    rospy.loginfo = lambda *args, **kwargs: None
    rospy.logwarn = lambda *args, **kwargs: None
    sys.modules["rospy"] = rospy

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.Joy = _Joy
    sensor_msgs_msg.Range = _Message
    sensor_msgs.msg = sensor_msgs_msg
    sys.modules["sensor_msgs"] = sensor_msgs
    sys.modules["sensor_msgs.msg"] = sensor_msgs_msg

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Bool = _Message
    std_msgs_msg.Float32 = _Message
    std_msgs_msg.String = _Message
    std_msgs_msg.UInt32 = _Message
    std_msgs.msg = std_msgs_msg
    sys.modules["std_msgs"] = std_msgs
    sys.modules["std_msgs.msg"] = std_msgs_msg

    source = Path(__file__).parents[1] / "scripts" / "virtual_driver_node.py"
    spec = importlib.util.spec_from_file_location("virtual_driver_under_test", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


DRIVER = _load_driver_module()


def _driver(robot_name="lucas"):
    """Build the policy portion of a driver without ROS publishers/subscribers."""
    driver = DRIVER.VirtualDriver.__new__(DRIVER.VirtualDriver)
    peer = "daisy" if robot_name == "lucas" else "lucas"
    driver.robot_name = robot_name
    driver.other_robots = [peer]
    driver.mode = DRIVER.DrivingMode.AGGRESSIVE
    driver.global_brake = False
    driver.power_level = 75.0
    driver.aggressive_battery_per_lap_estimate = 70.0
    driver.aggressive_default_lap_energy = 70.0
    driver.aggressive_battery_reserve = 10.0
    driver.aggressive_charge_start_margin = 12.0
    driver.aggressive_charge_target_margin = 5.0
    driver.aggressive_min_charge_gain = 1.0
    driver.aggressive_calibration_min_power = 80.0
    driver.aggressive_calibration_charge_target = 95.0
    driver.aggressive_full_charge_peer_safe_laps = 1
    driver.aggressive_priority_epsilon = 2.0
    driver.battery_per_lap_learned = False
    driver.other_power = {peer: 100.0}
    driver.other_lap_energy = {peer: 70.0}
    driver.other_one_lap_required_power = {peer: 80.0}
    driver.other_charge_sessions = {peer: 0}
    driver.other_charge_requested = {peer: False}
    driver.other_charger_claimed = {peer: False}
    driver.other_charge_request_stamp = {peer: None}
    driver.other_charger_claim_stamp = {peer: None}
    driver.other_in_fuel = {peer: False}
    driver.other_in_charge_gate = {peer: False}
    driver.other_in_merge = {peer: False}
    driver.other_in_fuel_by_camera = {peer: {"cam": False}}
    driver.charge_camera_names = ["cam"]
    driver.in_charge_gate_by_camera = {"cam": False}
    driver.in_charge_gate = False
    driver.charge_gate_until = 0.0
    driver.charge_gate_hold_sec = 1.2
    driver.charge_gate_arbitration_sec = 0.35
    driver.charge_gate_arbitration_until = 0.0
    driver.brake_command_retry_sec = 0.8
    driver.charger_status_timeout_sec = 2.0
    driver.local_brake = False
    driver.in_fuel_zone = False
    driver.in_merge_zone = False
    driver.waiting_for_fuel = False
    driver.seeking_fuel = False
    driver.leaving_charge = False
    driver.waiting_for_merge = False
    driver.merge_release_sent = False
    driver.gate_release_sent = False
    driver.gate_release_last_sent = None
    driver.fuel_wait_brake_last_sent = None
    driver.charge_state = DRIVER.ChargeState.IDLE
    driver.active_charge_target = 95.0
    driver.charge_session_count = 0
    driver.lap_count = 0.0
    driver.last_lap_count = None
    driver.lap_start_power = 75.0
    driver.charged_since_lap_start = False
    driver.lap_energy_pub = _Publisher()
    driver.one_lap_required_power_pub = _Publisher()
    driver.charge_sessions_pub = _Publisher()
    driver.charge_request_pub = _Publisher()
    driver.charger_claim_pub = _Publisher()
    return driver


class VirtualDriverChargingTests(unittest.TestCase):
    def setUp(self):
        _Clock.now = 0.0

    def test_short_gate_pulse_is_held_through_a_control_tick(self):
        driver = _driver()
        driver._latch_charge_gate()
        _Clock.now = 0.5  # Default driver tick is at 0.5 s.
        self.assertTrue(driver._at_charge_gate())
        _Clock.now = 1.201
        self.assertFalse(driver._at_charge_gate())

    def test_simultaneous_gate_arrivals_wait_one_arbitration_tick(self):
        driver = _driver()
        driver._start_charge_gate_arbitration()
        self.assertTrue(driver._charge_gate_arbitrating())
        _Clock.now = 0.5
        self.assertFalse(driver._charge_gate_arbitrating())

    def test_equal_simultaneous_arrivals_admit_only_the_tie_break_winner(self):
        lucas = _driver("lucas")
        daisy = _driver("daisy")
        strategy = DRIVER.AggressiveStrategy()
        for driver in (lucas, daisy):
            driver.power_level = 66.0
            driver.in_charge_gate = True
            strategy.decide(driver, DRIVER.DriverActions(driver))
            self.assertTrue(driver.waiting_for_fuel)

        # Model the status messages exchanged during the arbitration tick.
        lucas.other_power["daisy"] = daisy.power_level
        lucas.other_charge_requested["daisy"] = True
        lucas.other_charger_claimed["daisy"] = True
        daisy.other_power["lucas"] = lucas.power_level
        daisy.other_charge_requested["lucas"] = True
        daisy.other_charger_claimed["lucas"] = True
        lucas.local_brake = True
        daisy.local_brake = True
        _Clock.now = 0.5

        strategy.decide(lucas, DRIVER.DriverActions(lucas))
        strategy.decide(daisy, DRIVER.DriverActions(daisy))
        self.assertTrue(lucas.waiting_for_fuel)
        self.assertFalse(lucas.seeking_fuel)
        self.assertFalse(daisy.waiting_for_fuel)
        self.assertTrue(daisy.seeking_fuel)

    def test_conservative_prior_requests_charge_at_observed_first_gate_power(self):
        strategy = DRIVER.AggressiveStrategy()
        driver = _driver()
        driver.power_level = 66.0
        self.assertTrue(strategy.wants_charge(driver))
        driver.power_level = 81.0
        self.assertFalse(strategy.wants_charge(driver))

    def test_clean_lap_learns_energy_but_a_charge_contaminated_lap_does_not(self):
        driver = _driver()
        driver.last_lap_count = 0.0
        driver.lap_start_power = 100.0
        driver.power_level = 40.0
        driver._cb_lap_count(DRIVER.Float32(data=1.0))
        self.assertTrue(driver.battery_per_lap_learned)
        self.assertEqual(60.0, driver.aggressive_battery_per_lap_estimate)
        self.assertEqual(70.0, driver.aggressive_required_power())

        driver.lap_start_power = 100.0
        driver.power_level = 20.0
        driver.charged_since_lap_start = True
        driver._cb_lap_count(DRIVER.Float32(data=2.0))
        self.assertEqual(60.0, driver.aggressive_battery_per_lap_estimate)

    def test_contention_uses_one_lap_target_and_solo_charge_can_be_full(self):
        driver = _driver()
        peer = driver.other_robots[0]
        driver.battery_per_lap_learned = True
        driver.aggressive_battery_per_lap_estimate = 20.0
        driver.power_level = 25.0
        driver.other_lap_energy[peer] = 20.0
        driver.other_one_lap_required_power[peer] = 30.0
        driver.other_power[peer] = 25.0
        driver.other_charge_requested[peer] = True
        driver.other_charger_claimed[peer] = True
        self.assertEqual(35.0, driver.aggressive_charge_target_power())

        driver.other_charge_requested[peer] = False
        driver.other_charger_claimed[peer] = False
        driver.other_power[peer] = 90.0
        self.assertEqual(95.0, driver.aggressive_charge_target_power())

    def test_unlearned_contention_uses_the_conservative_one_lap_slot(self):
        driver = _driver()
        peer = driver.other_robots[0]
        driver.other_charge_requested[peer] = True
        driver.other_charger_claimed[peer] = True
        self.assertEqual(85.0, driver.aggressive_charge_target_power())

    def test_early_start_band_does_not_take_a_slot_without_adding_charge(self):
        driver = _driver()
        driver.battery_per_lap_learned = True
        driver.aggressive_battery_per_lap_estimate = 70.0
        driver.power_level = 90.0
        self.assertEqual(85.0, driver.aggressive_charge_target_power())
        self.assertFalse(driver.aggressive_charge_demand())
        self.assertFalse(DRIVER.AggressiveStrategy().wants_charge(driver))

    def test_peer_priority_uses_peer_budget_and_rotates_equal_margins(self):
        driver = _driver()
        peer = driver.other_robots[0]
        driver.battery_per_lap_learned = True
        driver.aggressive_battery_per_lap_estimate = 60.0
        driver.power_level = 65.0       # -5 % against Lucas's 70 % requirement
        driver.other_one_lap_required_power[peer] = 50.0
        driver.other_power[peer] = 45.0  # -5 % against Daisy's own requirement

        driver.charge_session_count = 1
        driver.other_charge_sessions[peer] = 0
        self.assertTrue(driver._aggressive_peer_has_priority(peer))

        driver.charge_session_count = 0
        driver.other_charge_sessions[peer] = 1
        self.assertFalse(driver._aggressive_peer_has_priority(peer))

    def test_waiting_car_still_sends_a_brake_command(self):
        driver = _driver()
        driver.waiting_for_fuel = True
        message = driver._charge_gate_wait_msg()
        self.assertEqual(1, message.buttons[DRIVER.BTN_X])

    def test_finished_charge_exits_before_reconsidering_start_threshold(self):
        driver = _driver()
        driver.battery_per_lap_learned = True
        driver.aggressive_battery_per_lap_estimate = 70.0
        driver.power_level = 85.0  # One-lap target, below the 92 % start level.
        driver.in_fuel_zone = True
        driver.leaving_charge = True
        message = DRIVER.AggressiveStrategy().decide(
            driver, DRIVER.DriverActions(driver))
        self.assertEqual(DRIVER.ChargeState.IDLE, driver.charge_state)
        self.assertEqual(1, message.buttons[DRIVER.BTN_B])


if __name__ == "__main__":
    unittest.main()
