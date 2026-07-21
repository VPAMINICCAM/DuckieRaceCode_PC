#!/usr/bin/env python3
"""Hardware-free regression tests for virtual-driver anti-collision control."""

import importlib.util
import sys
import threading
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
    """Load the ROS node with only the message surface these tests require."""
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
    spec = importlib.util.spec_from_file_location(
        "virtual_driver_anticollision_under_test", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


DRIVER = _load_driver_module()


def _driver(mode=DRIVER.DrivingMode.CONSERVATIVE):
    """Build policy state without starting ROS publishers or subscribers."""
    driver = DRIVER.VirtualDriver.__new__(DRIVER.VirtualDriver)
    driver.robot_name = "lucas"
    driver.other_robots = ["daisy"]
    driver.mode = mode

    driver.global_brake = False
    driver.local_brake = False
    driver.driver_brake_active = False
    driver.brake_released = True
    driver.power_level = 100.0
    driver.current_speed = DRIVER.SPEED_START
    driver.speed_offset = 0

    driver.tof_range = 9.9
    driver.front_range_valid = True
    driver.front_range_stamp = _Clock.now
    driver.front_closing_speed = 0.0
    driver.front_ttc = float("inf")

    # Conservative reacts sooner and leaves a larger restart gap.
    driver.conservative_safe_distance = 0.60
    driver.conservative_emergency_distance = 0.30
    driver.conservative_resume_distance = 0.42
    driver.conservative_ttc_slow = 3.0
    driver.conservative_ttc_brake = 1.2

    driver.aggressive_safe_distance = 0.45
    driver.aggressive_emergency_distance = 0.18
    driver.aggressive_resume_distance = 0.28
    driver.aggressive_ttc_slow = 2.0
    driver.aggressive_ttc_brake = 0.8

    driver.front_range_stale_sec = 1.0
    driver.front_closing_alpha = 0.35
    driver.anticollision_command_retry_sec = 0.8
    driver.anticollision_enabled = True
    driver.anticollision_lock = threading.RLock()
    driver.anticollision_sensor_required = False
    driver.anticollision_stop_latched = False
    driver.anticollision_brake_active = False
    driver.anticollision_brake_command_pending = False
    driver.anticollision_brake_last_sent = None
    driver.anticollision_release_pending = False
    driver.anticollision_state = "unavailable"

    driver.in_fuel_zone = False
    driver.in_charge_gate = False
    driver.in_merge_zone = False
    driver.tag_visible = False
    driver.in_charge_gate_by_camera = {"cam": False}
    driver.camera_names = ["cam"]
    driver.charge_camera_names = ["cam"]
    driver.charge_gate_until = 0.0
    driver.charge_gate_hold_sec = 1.2
    driver.charge_gate_arbitration_until = 0.0
    driver.charge_gate_arbitration_sec = 0.35

    driver.other_power = {"daisy": 100.0}
    driver.other_lap_energy = {"daisy": 70.0}
    driver.other_one_lap_required_power = {"daisy": 80.0}
    driver.other_charge_sessions = {"daisy": 0}
    driver.other_charge_requested = {"daisy": False}
    driver.other_charger_claimed = {"daisy": False}
    driver.other_charge_request_stamp = {"daisy": None}
    driver.other_charger_claim_stamp = {"daisy": None}
    driver.other_in_fuel = {"daisy": False}
    driver.other_in_charge_gate = {"daisy": False}
    driver.other_in_merge = {"daisy": False}
    driver.other_in_fuel_by_camera = {"daisy": {"cam": False}}

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

    driver.charge_state = DRIVER.ChargeState.IDLE
    driver.charge_waiting_for_collision_clear = False
    driver.active_charge_target = 95.0
    driver.active_charge_target_reason = ""
    driver.charge_session_count = 0
    driver.charge_lock_last_sent = None
    driver.brake_command_retry_sec = 0.8
    driver.charger_status_timeout_sec = 2.0
    driver.waiting_for_fuel = False
    driver.seeking_fuel = False
    driver.leaving_charge = False
    driver.waiting_for_merge = False
    driver.merge_release_sent = False
    driver.gate_release_sent = False
    driver.gate_release_last_sent = None
    driver.fuel_wait_brake_last_sent = None

    driver.lap_count = 0.0
    driver.last_lap_count = None
    driver.lap_start_power = driver.power_level
    driver.charged_since_lap_start = False

    driver.joy_pub = _Publisher()
    driver.collision_brake_pub = _Publisher()
    driver.mode_pub = _Publisher()
    driver.lap_energy_pub = _Publisher()
    driver.one_lap_required_power_pub = _Publisher()
    driver.charge_sessions_pub = _Publisher()
    driver.charge_request_pub = _Publisher()
    driver.charger_claim_pub = _Publisher()
    driver.actions = DRIVER.DriverActions(driver)
    return driver


def _button(message, index):
    if message is None or index >= len(message.buttons):
        return 0
    return message.buttons[index]


class VirtualDriverAntiCollisionTests(unittest.TestCase):
    def setUp(self):
        _Clock.now = 0.0

    def test_conservative_stops_at_a_distance_where_aggressive_only_slows(self):
        conservative = _driver(DRIVER.DrivingMode.CONSERVATIVE)
        aggressive = _driver(DRIVER.DrivingMode.AGGRESSIVE)
        for driver in (conservative, aggressive):
            driver.tof_range = 0.25
        aggressive.current_speed = DRIVER.SPEED_MAX

        self.assertEqual(
            "brake",
            conservative._front_safety_state(DRIVER.DrivingMode.CONSERVATIVE),
        )
        self.assertEqual(
            "slow",
            aggressive._front_safety_state(DRIVER.DrivingMode.AGGRESSIVE),
        )

        conservative_msg = DRIVER.ConservativeStrategy().decide(
            conservative, conservative.actions)
        aggressive_msg = DRIVER.AggressiveStrategy().decide(
            aggressive, aggressive.actions)
        self.assertEqual(0, _button(conservative_msg, DRIVER.BTN_X))
        self.assertTrue(conservative.collision_brake_pub.messages[-1].data)
        self.assertEqual(0, _button(aggressive_msg, DRIVER.BTN_X))
        self.assertEqual([], aggressive.collision_brake_pub.messages)
        self.assertEqual(1, _button(aggressive_msg, DRIVER.BTN_B))
        self.assertEqual(1, _button(aggressive_msg, DRIVER.BTN_R1))

    def test_conservative_crawls_inside_its_slow_distance(self):
        driver = _driver(DRIVER.DrivingMode.CONSERVATIVE)
        driver.tof_range = 0.50
        driver.current_speed = DRIVER.SPEED_START

        message = DRIVER.ConservativeStrategy().decide(
            driver, driver.actions)

        self.assertEqual("slow", driver.anticollision_state)
        self.assertEqual(1, _button(message, DRIVER.BTN_R1))
        self.assertEqual([], driver.collision_brake_pub.messages)

    def test_ttc_can_brake_before_the_emergency_distance(self):
        driver = _driver(DRIVER.DrivingMode.AGGRESSIVE)
        driver.tof_range = 0.80
        driver.front_closing_speed = 0.40
        driver.front_ttc = 0.60

        self.assertEqual(
            "brake",
            driver._front_safety_state(DRIVER.DrivingMode.AGGRESSIVE),
        )
        message = DRIVER.AggressiveStrategy().decide(driver, driver.actions)
        self.assertEqual(0, _button(message, DRIVER.BTN_X))
        self.assertTrue(driver.collision_brake_pub.messages[-1].data)

    def test_brake_bool_is_retried_without_a_joy_toggle(self):
        driver = _driver(DRIVER.DrivingMode.CONSERVATIVE)
        driver.tof_range = 0.10

        first = driver._anti_collision_override(driver.mode, driver.actions)
        _Clock.now = 0.10
        second = driver._anti_collision_override(driver.mode, driver.actions)

        self.assertEqual(0, _button(first, DRIVER.BTN_X))
        self.assertEqual(0, _button(second, DRIVER.BTN_X))
        self.assertEqual(1, len(driver.collision_brake_pub.messages))
        self.assertTrue(driver.collision_brake_pub.messages[0].data)
        self.assertTrue(driver.anticollision_brake_active)
        self.assertTrue(driver.anticollision_brake_command_pending)

        # A timeout safely republishes the same desired Bool state; it never
        # emits a second joystick toggle that could unlock the brake.
        _Clock.now = 0.90
        retry = driver._anti_collision_override(driver.mode, driver.actions)
        self.assertEqual(0, _button(retry, DRIVER.BTN_X))
        self.assertEqual(2, len(driver.collision_brake_pub.messages))
        self.assertTrue(driver.collision_brake_pub.messages[-1].data)

    def test_owned_brake_is_released_after_a_fresh_clear_measurement(self):
        driver = _driver(DRIVER.DrivingMode.CONSERVATIVE)
        driver.tof_range = 0.10

        lock = driver._anti_collision_override(driver.mode, driver.actions)
        self.assertEqual(0, _button(lock, DRIVER.BTN_X))
        self.assertTrue(driver.collision_brake_pub.messages[-1].data)
        driver._cb_local_brake(DRIVER.Bool(data=True))
        driver.tof_range = 0.80

        release = driver._anti_collision_override(driver.mode, driver.actions)

        self.assertEqual(0, _button(release, DRIVER.BTN_X))
        self.assertFalse(driver.collision_brake_pub.messages[-1].data)
        driver._cb_local_brake(DRIVER.Bool(data=False))
        self.assertFalse(driver.anticollision_brake_active)

    def test_release_is_retried_if_the_gap_settles_in_slow_band(self):
        driver = _driver(DRIVER.DrivingMode.CONSERVATIVE)
        driver.local_brake = True
        driver.anticollision_brake_active = True
        driver.anticollision_stop_latched = True
        driver.tof_range = 0.45

        driver._anti_collision_override(driver.mode, driver.actions)
        self.assertFalse(driver.collision_brake_pub.messages[-1].data)

        _Clock.now = driver.anticollision_command_retry_sec + 0.01
        driver.front_range_stamp = _Clock.now
        driver._anti_collision_override(driver.mode, driver.actions)

        self.assertEqual("slow", driver.anticollision_state)
        self.assertFalse(driver.collision_brake_pub.messages[-1].data)
        self.assertEqual(2, len(driver.collision_brake_pub.messages))

    def test_resume_hysteresis_is_independent_of_brake_ownership(self):
        driver = _driver(DRIVER.DrivingMode.CONSERVATIVE)
        driver.tof_range = 0.10
        self.assertEqual(
            "brake", driver._front_safety_state(driver.mode))
        self.assertTrue(driver.anticollision_stop_latched)

        # Above stop (0.30) but below resume (0.42) remains stopped.
        driver.tof_range = 0.35
        self.assertEqual(
            "brake", driver._front_safety_state(driver.mode))

        # Above resume but still inside slow (0.60) releases into crawl mode.
        driver.tof_range = 0.45
        self.assertEqual(
            "clear", driver._front_safety_state(driver.mode))
        self.assertFalse(driver.anticollision_stop_latched)
        self.assertEqual(
            "slow", driver._front_safety_state(driver.mode))

    def test_clear_path_does_not_release_an_unrelated_brake(self):
        for mode, strategy in (
            (DRIVER.DrivingMode.CONSERVATIVE, DRIVER.ConservativeStrategy()),
            (DRIVER.DrivingMode.AGGRESSIVE, DRIVER.AggressiveStrategy()),
        ):
            with self.subTest(mode=mode.value):
                driver = _driver(mode)
                driver.local_brake = True
                driver.tof_range = 1.0

                override = driver._anti_collision_override(mode, driver.actions)
                message = strategy.decide(driver, driver.actions)

                self.assertEqual(0, _button(override, DRIVER.BTN_X))
                self.assertEqual(0, _button(message, DRIVER.BTN_X))
                self.assertFalse(driver.anticollision_brake_active)
                self.assertEqual([], driver.collision_brake_pub.messages)

    def test_hazard_asserts_owner_even_if_effective_brake_looks_engaged(self):
        driver = _driver(DRIVER.DrivingMode.CONSERVATIVE)
        # This can be a stale /local_brake value after the policy or robot
        # process restarts.  Hazard enforcement must not rely on it.
        driver.local_brake = True
        driver.driver_brake_active = False
        driver.tof_range = 0.10

        driver._anti_collision_override(driver.mode, driver.actions)

        self.assertTrue(driver.anticollision_brake_active)
        self.assertTrue(driver.collision_brake_pub.messages[-1].data)

    def test_stale_or_invalid_range_holds_an_owned_brake(self):
        cases = ("stale", "invalid")
        for case in cases:
            with self.subTest(case=case):
                driver = _driver(DRIVER.DrivingMode.AGGRESSIVE)
                driver.local_brake = True
                driver.anticollision_brake_active = True
                driver.tof_range = 1.0
                if case == "stale":
                    _Clock.now = driver.front_range_stale_sec + 0.01
                else:
                    driver.front_range_valid = False

                message = driver._anti_collision_override(
                    DRIVER.DrivingMode.AGGRESSIVE, driver.actions)

                self.assertEqual(0, _button(message, DRIVER.BTN_X))
                self.assertTrue(driver.anticollision_brake_active)
                self.assertEqual("unavailable", driver.anticollision_state)
                _Clock.now = 0.0

    def test_sensor_loss_cannot_release_existing_gate_brake_after_hazard(self):
        driver = _driver(DRIVER.DrivingMode.CONSERVATIVE)
        driver.power_level = 50.0
        driver.in_charge_gate = True
        driver.local_brake = True
        driver.tof_range = 0.10
        strategy = DRIVER.ConservativeStrategy()

        strategy.decide(driver, driver.actions)
        self.assertTrue(driver.anticollision_stop_latched)
        driver.front_range_valid = False
        message = strategy.decide(driver, driver.actions)

        self.assertEqual(0, _button(message, DRIVER.BTN_X))
        self.assertFalse(driver.seeking_fuel)
        self.assertTrue(driver.anticollision_stop_latched)

    def test_startup_does_not_unlock_into_a_fresh_hazard(self):
        driver = _driver(DRIVER.DrivingMode.CONSERVATIVE)
        driver.brake_released = False
        driver.local_brake = True
        driver.driver_brake_active = True
        driver.tof_range = 0.10

        driver._control_loop(None)

        self.assertEqual("brake", driver.anticollision_state)
        self.assertTrue(driver.anticollision_brake_active)
        self.assertFalse(driver.brake_released)
        self.assertTrue(driver.collision_brake_pub.messages[-1].data)
        self.assertTrue(all(
            _button(message, DRIVER.BTN_X) == 0
            for message in driver.joy_pub.messages
        ))

        driver.tof_range = 0.35
        driver._control_loop(None)
        self.assertFalse(driver.brake_released)
        self.assertTrue(all(
            _button(message, DRIVER.BTN_X) == 0
            for message in driver.joy_pub.messages
        ))

        driver.tof_range = 0.45
        driver._control_loop(None)
        self.assertFalse(driver.brake_released)
        self.assertFalse(driver.collision_brake_pub.messages[-1].data)
        driver._cb_collision_brake_status(DRIVER.Bool(data=False))
        driver._control_loop(None)
        self.assertTrue(driver.brake_released)
        self.assertEqual(1, _button(
            driver.joy_pub.messages[-1], DRIVER.BTN_X))

    def test_startup_preserves_an_acknowledged_collision_owner(self):
        driver = _driver(DRIVER.DrivingMode.CONSERVATIVE)
        driver.brake_released = False
        driver.local_brake = True
        driver.tof_range = 0.10
        driver.anticollision_brake_active = True
        driver.anticollision_stop_latched = True

        driver._control_loop(None)

        self.assertTrue(driver.brake_released)
        self.assertTrue(driver.anticollision_brake_active)
        self.assertTrue(all(
            _button(message, DRIVER.BTN_X) == 0
            for message in driver.joy_pub.messages
        ))

        driver.tof_range = 0.80
        driver._control_loop(None)
        self.assertFalse(driver.collision_brake_pub.messages[-1].data)
        self.assertTrue(all(
            _button(message, DRIVER.BTN_X) == 0
            for message in driver.joy_pub.messages
        ))

    def test_startup_releases_both_overlapping_brake_owners_in_order(self):
        driver = _driver(DRIVER.DrivingMode.CONSERVATIVE)
        driver.brake_released = False
        driver.local_brake = True
        driver.driver_brake_active = True
        driver.tof_range = 0.10
        driver.anticollision_brake_active = True
        driver.anticollision_stop_latched = True

        driver._control_loop(None)
        self.assertFalse(driver.brake_released)

        driver.tof_range = 0.80
        driver._control_loop(None)
        self.assertFalse(driver.collision_brake_pub.messages[-1].data)
        self.assertFalse(driver.brake_released)

        driver._cb_collision_brake_status(DRIVER.Bool(data=False))
        driver._control_loop(None)
        self.assertTrue(driver.brake_released)
        self.assertEqual(1, _button(
            driver.joy_pub.messages[-1], DRIVER.BTN_X))

    def test_robot_restart_reenters_driver_brake_handshake(self):
        for order in ("local_first", "owner_first"):
            with self.subTest(order=order):
                driver = _driver(DRIVER.DrivingMode.AGGRESSIVE)
                driver.brake_released = True
                callbacks = (
                    (
                        driver._cb_local_brake,
                        driver._cb_driver_brake_status,
                    )
                    if order == "local_first"
                    else (
                        driver._cb_driver_brake_status,
                        driver._cb_local_brake,
                    )
                )
                callbacks[0](DRIVER.Bool(data=True))
                callbacks[1](DRIVER.Bool(data=True))

                self.assertFalse(driver.brake_released)
                driver._control_loop(None)
                self.assertTrue(driver.brake_released)
                self.assertEqual(1, _button(
                    driver.joy_pub.messages[-1], DRIVER.BTN_X))

    def test_charge_completion_keeps_brake_locked_when_front_is_blocked(self):
        driver = _driver(DRIVER.DrivingMode.CONSERVATIVE)
        driver.charge_state = DRIVER.ChargeState.CHARGING
        driver.local_brake = True
        driver.in_fuel_zone = True
        driver.power_level = 95.0
        driver.tof_range = 0.10

        completion = driver._charge_control_msg(target_power=95.0)

        self.assertEqual(DRIVER.ChargeState.CHARGING, driver.charge_state)
        self.assertTrue(driver.charge_waiting_for_collision_clear)
        self.assertEqual(0, _button(completion, DRIVER.BTN_X))
        self.assertEqual(1, _button(completion, DRIVER.BTN_B))
        self.assertEqual([], driver.collision_brake_pub.messages)

        driver.tof_range = 0.45
        release = driver._charge_control_msg(target_power=95.0)
        self.assertEqual(DRIVER.ChargeState.IDLE, driver.charge_state)
        self.assertEqual(1, _button(release, DRIVER.BTN_X))
        self.assertEqual(1, _button(release, DRIVER.BTN_B))
        self.assertEqual([], driver.collision_brake_pub.messages)

    def test_range_callback_brakes_at_sensor_rate(self):
        driver = _driver(DRIVER.DrivingMode.AGGRESSIVE)
        driver._cb_tof(DRIVER.Range(range=0.10, min_range=0.05))

        self.assertEqual("brake", driver.anticollision_state)
        self.assertTrue(driver.anticollision_brake_active)
        self.assertTrue(driver.collision_brake_pub.messages[-1].data)

    def test_range_callback_also_guards_the_charging_lock_transition(self):
        driver = _driver(DRIVER.DrivingMode.CONSERVATIVE)
        driver.charge_state = DRIVER.ChargeState.LOCKING

        driver._cb_tof(DRIVER.Range(range=0.10, min_range=0.05))

        self.assertTrue(driver.anticollision_brake_active)
        self.assertTrue(driver.collision_brake_pub.messages[-1].data)
        message = DRIVER.ConservativeStrategy().decide(
            driver, driver.actions)
        self.assertEqual(0, _button(message, DRIVER.BTN_X))

    def test_newer_range_update_cannot_be_overwritten_by_stale_release(self):
        driver = _driver(DRIVER.DrivingMode.CONSERVATIVE)
        driver.local_brake = True
        driver.anticollision_brake_active = True
        driver.anticollision_stop_latched = True
        driver.tof_range = 0.80
        entered = threading.Event()
        continue_release = threading.Event()
        sensor_done = threading.Event()
        original_classifier = driver._front_safety_state

        def delayed_clear(_profile, allow_release=True):
            entered.set()
            self.assertTrue(continue_release.wait(1.0))
            return "clear"

        driver._front_safety_state = delayed_clear
        release_thread = threading.Thread(
            target=driver._anti_collision_override,
            args=(driver.mode, driver.actions),
        )
        release_thread.start()
        self.assertTrue(entered.wait(1.0))

        def deliver_hazard():
            driver._cb_tof(DRIVER.Range(range=0.10, min_range=0.05))
            sensor_done.set()

        sensor_thread = threading.Thread(target=deliver_hazard)
        sensor_thread.start()
        self.assertFalse(sensor_done.wait(0.05))
        driver._front_safety_state = original_classifier
        continue_release.set()
        release_thread.join(1.0)
        sensor_thread.join(1.0)

        self.assertFalse(release_thread.is_alive())
        self.assertFalse(sensor_thread.is_alive())
        self.assertFalse(driver.collision_brake_pub.messages[-2].data)
        self.assertTrue(driver.collision_brake_pub.messages[-1].data)

    def test_sensor_rate_path_defers_release_to_policy_loop(self):
        driver = _driver(DRIVER.DrivingMode.CONSERVATIVE)
        driver.local_brake = True
        driver.in_fuel_zone = True
        driver.leaving_charge = True
        driver.anticollision_brake_active = True
        driver.anticollision_stop_latched = True
        driver.tof_range = 0.10

        driver._cb_tof(DRIVER.Range(range=0.80))
        self.assertTrue(driver.anticollision_brake_active)
        self.assertTrue(driver.anticollision_stop_latched)
        self.assertEqual([], driver.collision_brake_pub.messages)

        release = driver._anti_collision_override(driver.mode, driver.actions)
        self.assertFalse(driver.collision_brake_pub.messages[-1].data)
        self.assertEqual(1, _button(release, DRIVER.BTN_B))

    def test_required_sensor_watchdog_stops_on_missing_data(self):
        driver = _driver(DRIVER.DrivingMode.CONSERVATIVE)
        driver.anticollision_sensor_required = True
        driver.front_range_valid = False
        driver.front_range_stamp = None

        driver._safety_watchdog(None)

        self.assertEqual("brake", driver.anticollision_state)
        self.assertTrue(driver.anticollision_stop_latched)
        self.assertTrue(driver.collision_brake_pub.messages[-1].data)

    def test_optional_sensor_can_be_absent(self):
        driver = _driver(DRIVER.DrivingMode.CONSERVATIVE)
        driver.front_range_valid = False
        driver.front_range_stamp = None

        driver._safety_watchdog(None)

        self.assertEqual("unavailable", driver.anticollision_state)
        self.assertEqual([], driver.collision_brake_pub.messages)

    def test_invalid_sample_does_not_create_false_closing_speed(self):
        driver = _driver(DRIVER.DrivingMode.AGGRESSIVE)
        driver._cb_tof(DRIVER.Range(range=float("inf")))
        self.assertFalse(driver.front_range_valid)

        _Clock.now = 0.10
        driver._cb_tof(DRIVER.Range(range=0.80))
        self.assertTrue(driver.front_range_valid)
        self.assertEqual(0.0, driver.front_closing_speed)
        self.assertEqual(float("inf"), driver.front_ttc)
        self.assertEqual([], driver.collision_brake_pub.messages)

    def test_rep117_too_close_range_is_an_emergency(self):
        driver = _driver(DRIVER.DrivingMode.AGGRESSIVE)
        driver._cb_tof(DRIVER.Range(
            range=float("-inf"), min_range=0.05))

        self.assertTrue(driver.front_range_valid)
        self.assertEqual(0.05, driver.tof_range)
        self.assertTrue(driver.collision_brake_pub.messages[-1].data)

    def test_collision_status_is_reconciled_after_driver_restart(self):
        driver = _driver(DRIVER.DrivingMode.CONSERVATIVE)
        driver.local_brake = True
        driver._cb_collision_brake_status(DRIVER.Bool(data=True))
        self.assertTrue(driver.anticollision_brake_active)
        self.assertTrue(driver.anticollision_stop_latched)

        driver.tof_range = 1.0
        driver._anti_collision_override(driver.mode, driver.actions)
        self.assertFalse(driver.collision_brake_pub.messages[-1].data)
        driver._cb_collision_brake_status(DRIVER.Bool(data=False))
        self.assertFalse(driver.anticollision_brake_active)

    def test_clear_command_does_not_trust_a_lagging_effective_brake_topic(self):
        driver = _driver(DRIVER.DrivingMode.CONSERVATIVE)
        driver.local_brake = False
        driver.tof_range = 1.0
        driver._cb_collision_brake_status(DRIVER.Bool(data=True))

        driver._anti_collision_override(driver.mode, driver.actions)

        self.assertTrue(driver.anticollision_brake_active)
        self.assertTrue(driver.anticollision_release_pending)
        self.assertFalse(driver.collision_brake_pub.messages[-1].data)
        driver._cb_collision_brake_status(DRIVER.Bool(data=False))
        self.assertFalse(driver.anticollision_brake_active)

    def test_manual_mode_relinquishes_owned_collision_brake(self):
        driver = _driver(DRIVER.DrivingMode.CONSERVATIVE)
        driver.anticollision_brake_active = True
        driver.anticollision_stop_latched = True

        driver._cb_set_mode(DRIVER.String(data="manual"))

        self.assertEqual(DRIVER.DrivingMode.MANUAL, driver.mode)
        self.assertFalse(driver.anticollision_brake_active)
        self.assertFalse(driver.anticollision_stop_latched)
        self.assertFalse(driver.collision_brake_pub.messages[-1].data)

    def test_manual_start_clears_orphaned_robot_collision_status(self):
        driver = _driver(DRIVER.DrivingMode.MANUAL)
        driver._cb_collision_brake_status(DRIVER.Bool(data=True))

        self.assertFalse(driver.anticollision_brake_active)
        self.assertFalse(driver.collision_brake_pub.messages[-1].data)


if __name__ == "__main__":
    unittest.main()
