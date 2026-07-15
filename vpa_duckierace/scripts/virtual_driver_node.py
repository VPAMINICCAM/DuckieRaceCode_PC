#!/usr/bin/env python3
"""
Virtual Driver Node for DuckieRace
====================================
Publishes synthetic Joy messages to autonomously control one robot using a
selectable driving-behavior mode.  Designed to run on the main PC.

Behavior Modes
--------------
MANUAL       - Virtual driver is inactive; physical joystick is used instead.
CONSERVATIVE - Low speed, charges whenever inside the fuel zone.
AGGRESSIVE   - Max speed, charges only when power is critically low;
               slows / overtakes based on front range and charges when the
               learned one-lap battery budget is no longer available.
COOPERATIVE  - Medium speed, yields at obstacles, shares the fuel zone fairly
               with the other robots.
ADAPTIVE     - Switches automatically between aggressive / conservative based
               on the current power level; rushes to the fuel zone when low.

ROS Interface
-------------
Subscribed topics (all absolute paths):
  /{robot_name}/power_level        Float32  Virtual power level (0-100 %)
  /{robot_name}/speed_percent      Float32  Robot-reported target speed
  /{robot_name}/local_brake        Bool     Current local brake state
  /{robot_name}/in_fuel_zone       Bool     Whether robot is in the fuel zone
  /{robot_name}/in_charge_gate_zone Bool    Whether robot is at charge-gate entry
  /{robot_name}/in_merge_zone      Bool     Whether robot is in the merge zone
  /{robot_name}/front_range        Range    ToF distance to obstacle ahead
  /{robot_name}/lap_count          Float32  Completed laps
  /{robot_name}/set_driving_mode   String   Runtime mode-change command
  /{other_robot}/power_level       Float32  Peer robots' power (cooperative mode)
  /{other_robot}/in_fuel_zone      Bool     Peer robots' fuel-zone occupancy
  /{other_robot}/in_charge_gate_zone Bool    Peer robots' charge-gate occupancy
  /{other_robot}/in_merge_zone     Bool     Peer robots' merge-zone occupancy
  /global_brake                    Bool     Race-level start / stop signal

Published topics:
  /{robot_name}/joy                Joy      Synthetic joystick commands
  /{robot_name}/driving_mode       String   Active mode name (latched)

Parameters
----------
~robot_name    str    default 'daisy'         Robot ROS namespace
~driving_mode  str    default 'conservative'  Initial behavior mode
~all_robots    list   default [lucas,daisy]
~control_rate  float  default 2.0             Control-loop frequency [Hz]
"""

import rospy
import math
from sensor_msgs.msg import Joy, Range
from std_msgs.msg import Bool, Float32, String
from enum import Enum


# ---------------------------------------------------------------------------
# Behaviour modes
# ---------------------------------------------------------------------------

class DrivingMode(Enum):
    MANUAL       = "manual"
    CONSERVATIVE = "conservative"
    AGGRESSIVE   = "aggressive"
    COOPERATIVE  = "cooperative"
    ADAPTIVE     = "adaptive"


class ChargeState(Enum):
    IDLE      = "idle"
    LOCKING   = "locking"
    CHARGING  = "charging"


# ---------------------------------------------------------------------------
# Joy button indices, matching robot-side duckierace.py.
# ---------------------------------------------------------------------------
BTN_X  = 0   # Brake toggle
BTN_B  = 2   # Yellow line / must hold to unlock brake inside fuel zone
BTN_Y  = 3   # Enable charging (while in fuel zone)
BTN_L1 = 4   # Increase speed by one step
BTN_R1 = 5   # Decrease speed by one step

# Speed parameters - keep in sync with duckierace.py
SPEED_START   = 0.25
SPEED_STEP    = 0.02
SPEED_MAX     = 0.35
SPEED_MIN     = 0.20
STEPS_TO_MAX  = int(round((SPEED_MAX - SPEED_START) / SPEED_STEP))  # 5
STEPS_TO_MIN  = -int(round((SPEED_START - SPEED_MIN) / SPEED_STEP)) # -2
STEPS_TO_HALF = 0  # 50% in the GUI speed scale maps to SPEED_START.

# Power thresholds
POWER_CRITICAL    = 15.0   # % - charge now regardless of mode
CONSERVATIVE_CHARGE_START = 55.0  # % - conservative robot starts seeking charge below this
POWER_CHARGE_FULL = 95.0   # % - stop charging above this
COOP_CHARGE_SELF  = 50.0   # % - cooperative robot charges below this
COOP_CHARGE_PEER  = 30.0   # % - cooperative robot yields if peer < this

# Obstacle distance [m]
OVERTAKE_DIST = 0.30        # activate yellow line to pass slow robot ahead
YIELD_DIST    = 0.25        # slow down (cooperative yield)

# Aggressive v1 tuning
AGGRESSIVE_SAFE_DISTANCE = 0.45          # [m] begin slowing / overtaking
AGGRESSIVE_EMERGENCY_DISTANCE = 0.18     # [m] lock brake if still closing
AGGRESSIVE_TTC_SLOW = 2.0                # [s] time-to-collision slow threshold
AGGRESSIVE_TTC_BRAKE = 0.8               # [s] time-to-collision brake threshold
AGGRESSIVE_RANGE_STALE_SEC = 1.0
AGGRESSIVE_CLOSING_ALPHA = 0.35
AGGRESSIVE_BATTERY_PER_LAP_DEFAULT = 20.0
AGGRESSIVE_BATTERY_RESERVE = 10.0
AGGRESSIVE_LAP_EMA_ALPHA = 0.35
AGGRESSIVE_MIN_VALID_LAP_DROP = 1.0
AGGRESSIVE_CHARGE_START_MARGIN = 12.0
AGGRESSIVE_DEFER_MARGIN = 6.0
AGGRESSIVE_PRIORITY_EPSILON = 2.0
AGGRESSIVE_PEER_LOW_POWER = 60.0
AGGRESSIVE_NORMAL_CHARGE_TARGET = 90.0
AGGRESSIVE_PRESSURE_CHARGE_TARGET = 55.0
AGGRESSIVE_URGENT_CHARGE_TARGET = 65.0
AGGRESSIVE_CHARGE_TARGET_MARGIN = 5.0


# ---------------------------------------------------------------------------
# Strategy interfaces
# ---------------------------------------------------------------------------

class DriverActions:
    def __init__(self, driver):
        self.driver = driver

    def idle(self) -> Joy:
        return self.driver._make_joy()

    def press(self, *button_indices) -> Joy:
        return self.driver._joy_press(*button_indices)

    def release_brake(self) -> Joy:
        return self.driver._release_brake_msg()

    def hold_yellow_line(self) -> Joy:
        return self.driver._charge_gate_control_msg()

    def slow_for_charge_gate(self) -> Joy:
        return self.driver._charge_gate_slow_msg()

    def wait_at_charge_gate(self) -> Joy:
        return self.driver._charge_gate_wait_msg()

    def wait_at_merge(self) -> Joy:
        return self.driver._merge_wait_msg()

    def release_from_merge(self):
        return self.driver._merge_release_msg()

    def charge(self, target_power: float, wait_for_merge: bool = True) -> Joy:
        return self.driver._charge_control_msg(target_power, wait_for_merge)

    def speed_step_msg(self, target_offset: int) -> Joy:
        buttons = [0] * 12
        self.driver._step_speed(target_offset, buttons)
        return self.driver._make_joy(buttons=buttons)


class DrivingStrategyBase:
    def wants_charge(self, driver) -> bool:
        return False

    def charge_target_power(self, driver) -> float:
        return POWER_CHARGE_FULL

    def waits_for_fuel_occupancy(self, driver) -> bool:
        return True

    def waits_for_merge_occupancy(self, driver) -> bool:
        return True

    def defers_charge_for_peer(self, driver) -> bool:
        return False

    def waits_for_charge_gate_priority(self, driver) -> bool:
        return False

    def normal_drive(self, driver, actions: DriverActions) -> Joy:
        return actions.idle()

    def decide(self, driver, actions: DriverActions) -> Joy:
        if driver.charge_state != ChargeState.IDLE:
            return actions.charge(
                self.charge_target_power(driver),
                self.waits_for_merge_occupancy(driver))

        if driver.seeking_fuel:
            if driver.in_fuel_zone:
                driver.seeking_fuel = False
            elif (self.waits_for_fuel_occupancy(driver)
                    and driver._active_charge_fuel_occupied_by_other()):
                driver.seeking_fuel = False
                driver.gate_release_sent = False
                return actions.wait_at_charge_gate()
            elif self.defers_charge_for_peer(driver):
                driver.seeking_fuel = False
                driver.gate_release_sent = False
                return self.normal_drive(driver, actions)
            else:
                return actions.slow_for_charge_gate()

        if driver.in_fuel_zone and self.wants_charge(driver):
            driver.charge_state = ChargeState.LOCKING
            return actions.charge(
                self.charge_target_power(driver),
                self.waits_for_merge_occupancy(driver))

        if driver.in_fuel_zone:
            if (self.waits_for_merge_occupancy(driver)
                    and driver._merge_zone_occupied_by_other()):
                driver.merge_release_sent = False
                return actions.wait_at_merge()
            if driver.waiting_for_merge:
                joy_msg = actions.release_from_merge()
                if joy_msg is not None:
                    return joy_msg

        if driver.leaving_charge:
            if driver.in_merge_zone:
                driver.leaving_charge = False
            else:
                return actions.hold_yellow_line()

        if (self.wants_charge(driver)
                and driver._at_charge_gate()
                and not driver.in_fuel_zone):
            if (self.waits_for_fuel_occupancy(driver)
                    and (driver._active_charge_fuel_occupied_by_other()
                         or self.waits_for_charge_gate_priority(driver))):
                driver.gate_release_sent = False
                return actions.wait_at_charge_gate()
            if driver.local_brake:
                driver.waiting_for_fuel = False
                if not driver.gate_release_sent:
                    driver.gate_release_sent = True
                    return actions.release_brake()
                return actions.idle()
            driver.waiting_for_fuel = False
            driver.gate_release_sent = False
            driver.seeking_fuel = True
            return actions.slow_for_charge_gate()

        driver.waiting_for_fuel = False
        driver.gate_release_sent = False
        return self.normal_drive(driver, actions)


class ConservativeStrategy(DrivingStrategyBase):
    def wants_charge(self, driver) -> bool:
        return driver.power_level < CONSERVATIVE_CHARGE_START

    def normal_drive(self, driver, actions: DriverActions) -> Joy:
        return actions.speed_step_msg(STEPS_TO_MIN)


class AggressiveStrategy(DrivingStrategyBase):
    def wants_charge(self, driver) -> bool:
        if driver.power_level < POWER_CRITICAL:
            return True
        if driver.power_level <= driver.aggressive_hard_charge_threshold():
            return True
        if driver.power_level <= driver.aggressive_charge_start_threshold():
            return not driver._aggressive_should_defer_charge()
        return False

    def charge_target_power(self, driver) -> float:
        return driver.aggressive_charge_target_power()

    def defers_charge_for_peer(self, driver) -> bool:
        return driver._aggressive_should_defer_charge()

    def waits_for_charge_gate_priority(self, driver) -> bool:
        return driver._aggressive_peer_has_gate_priority()

    def normal_drive(self, driver, actions: DriverActions) -> Joy:
        buttons = [0] * 12
        safety_state = driver._aggressive_front_safety_state()

        if safety_state == "brake":
            if not driver.local_brake:
                return actions.press(BTN_X)
            return actions.idle()

        if driver.local_brake:
            return actions.release_brake()

        target_offset = STEPS_TO_MAX
        if safety_state == "slow":
            target_offset = 0
            buttons[BTN_B] = 1
        elif safety_state == "overtake":
            buttons[BTN_B] = 1

        driver._step_speed(target_offset, buttons)
        return driver._make_joy(buttons=buttons)


class CooperativeStrategy(DrivingStrategyBase):
    def wants_charge(self, driver) -> bool:
        peers_not_critical = all(
            p > COOP_CHARGE_PEER for p in driver.other_power.values())
        return (
            driver.power_level < COOP_CHARGE_SELF
            and (peers_not_critical or driver.power_level < POWER_CRITICAL))

    def normal_drive(self, driver, actions: DriverActions) -> Joy:
        buttons = [0] * 12
        driver._step_speed(0, buttons)
        if driver.tof_range < YIELD_DIST:
            buttons[BTN_B] = 1
        return driver._make_joy(buttons=buttons)


class AdaptiveStrategy(DrivingStrategyBase):
    def wants_charge(self, driver) -> bool:
        return driver.power_level <= 60.0

    def normal_drive(self, driver, actions: DriverActions) -> Joy:
        if driver.power_level > 60.0:
            joy = AggressiveStrategy().normal_drive(driver, actions)
            if driver.in_fuel_zone and not driver.local_brake:
                joy.buttons[BTN_B] = 1
            return joy

        if driver.power_level > 20.0:
            return ConservativeStrategy().normal_drive(driver, actions)

        buttons = [0] * 12
        if driver.tag_visible:
            buttons[BTN_B] = 1
        return driver._make_joy(buttons=buttons)


STRATEGIES = {
    DrivingMode.CONSERVATIVE: ConservativeStrategy(),
    DrivingMode.AGGRESSIVE: AggressiveStrategy(),
    DrivingMode.COOPERATIVE: CooperativeStrategy(),
    DrivingMode.ADAPTIVE: AdaptiveStrategy(),
}


# ---------------------------------------------------------------------------
# Main node class
# ---------------------------------------------------------------------------

class VirtualDriver:
    def __init__(self):
        rospy.init_node("virtual_driver_node")

        self.robot_name  = rospy.get_param("~robot_name", "daisy")
        mode_str         = rospy.get_param("~driving_mode", "conservative")
        all_robots       = rospy.get_param(
            "~all_robots", ["lucas", "daisy"])
        self.camera_names = rospy.get_param(
            "~camera_names", ["usb_cam_1", "usb_cam_2"])
        self.charge_camera_names = rospy.get_param(
            "~charge_camera_names", ["usb_cam_1", "usb_cam_2"])
        self.control_rate = float(rospy.get_param("~control_rate", 2.0))
        self.aggressive_safe_distance = float(rospy.get_param(
            "~aggressive_safe_distance", AGGRESSIVE_SAFE_DISTANCE))
        self.aggressive_emergency_distance = float(rospy.get_param(
            "~aggressive_emergency_distance", AGGRESSIVE_EMERGENCY_DISTANCE))
        self.aggressive_ttc_slow = float(rospy.get_param(
            "~aggressive_ttc_slow", AGGRESSIVE_TTC_SLOW))
        self.aggressive_ttc_brake = float(rospy.get_param(
            "~aggressive_ttc_brake", AGGRESSIVE_TTC_BRAKE))
        self.aggressive_battery_reserve = float(rospy.get_param(
            "~aggressive_battery_reserve", AGGRESSIVE_BATTERY_RESERVE))
        self.aggressive_battery_per_lap_estimate = float(rospy.get_param(
            "~aggressive_battery_per_lap_default",
            AGGRESSIVE_BATTERY_PER_LAP_DEFAULT))
        self.aggressive_charge_start_margin = float(rospy.get_param(
            "~aggressive_charge_start_margin",
            AGGRESSIVE_CHARGE_START_MARGIN))
        self.aggressive_defer_margin = float(rospy.get_param(
            "~aggressive_defer_margin", AGGRESSIVE_DEFER_MARGIN))
        self.aggressive_priority_epsilon = float(rospy.get_param(
            "~aggressive_priority_epsilon",
            AGGRESSIVE_PRIORITY_EPSILON))
        self.aggressive_peer_low_power = float(rospy.get_param(
            "~aggressive_peer_low_power", AGGRESSIVE_PEER_LOW_POWER))
        self.aggressive_normal_charge_target = float(rospy.get_param(
            "~aggressive_normal_charge_target",
            AGGRESSIVE_NORMAL_CHARGE_TARGET))
        self.aggressive_pressure_charge_target = float(rospy.get_param(
            "~aggressive_pressure_charge_target",
            AGGRESSIVE_PRESSURE_CHARGE_TARGET))
        self.aggressive_urgent_charge_target = float(rospy.get_param(
            "~aggressive_urgent_charge_target",
            AGGRESSIVE_URGENT_CHARGE_TARGET))
        self.aggressive_charge_target_margin = float(rospy.get_param(
            "~aggressive_charge_target_margin",
            AGGRESSIVE_CHARGE_TARGET_MARGIN))

        try:
            self.mode = DrivingMode(mode_str.strip().lower())
        except ValueError:
            rospy.logwarn(
                f"[VirtualDriver/{self.robot_name}] "
                f"Unknown mode '{mode_str}', falling back to 'conservative'.")
            self.mode = DrivingMode.CONSERVATIVE

        self.other_robots = [r for r in all_robots if r != self.robot_name]

        # ---- Race state -------------------------------------------------------
        self.power_level       = 75.0
        self.in_fuel_zone      = False
        self.in_charge_gate    = False
        self.in_merge_zone     = False
        self.tag_visible       = False
        self.local_brake       = True
        self.global_brake      = True    # True = game not running
        self.tof_range         = 9.9     # metres - default "clear"
        self.current_speed     = SPEED_START
        self.front_range_stamp = None
        self.front_closing_speed = 0.0
        self.front_ttc         = float("inf")
        self.lap_count         = 0.0
        self.last_lap_count    = None
        self.lap_start_power   = self.power_level
        self.battery_per_lap_learned = False
        self.other_power       = {r: 75.0 for r in self.other_robots}
        self.other_in_fuel     = {r: False for r in self.other_robots}
        self.other_in_charge_gate = {r: False for r in self.other_robots}
        self.other_in_merge    = {r: False for r in self.other_robots}
        self.in_charge_gate_by_camera = {cam: False for cam in self.camera_names}
        self.other_in_fuel_by_camera = {
            r: {cam: False for cam in self.camera_names}
            for r in self.other_robots
        }

        # ---- Internal driver state -------------------------------------------
        # Have we already sent the one-shot brake-release Joy message?
        self.brake_released    = False
        # Current speed step offset, synchronised from /speed_percent when possible.
        # +N means N * L1 presses sent, -N means N * R1 presses sent.
        self.speed_offset      = 0
        self.charge_state      = ChargeState.IDLE
        self.waiting_for_fuel  = False
        self.gate_release_sent = False
        self.waiting_for_merge = False
        self.merge_release_sent = False
        self.seeking_fuel      = False
        self.leaving_charge    = False
        self.actions           = DriverActions(self)

        # ---- Publishers -------------------------------------------------------
        self.joy_pub  = rospy.Publisher(
            f"/{self.robot_name}/joy", Joy, queue_size=1)
        self.mode_pub = rospy.Publisher(
            f"/{self.robot_name}/driving_mode", String, queue_size=1, latch=True)

        # ---- Subscribers ------------------------------------------------------
        rospy.Subscriber(f"/{self.robot_name}/power_level",
                         Float32, self._cb_power)
        rospy.Subscriber(f"/{self.robot_name}/speed_percent",
                         Float32, self._cb_speed)
        rospy.Subscriber(f"/{self.robot_name}/local_brake",
                         Bool,    self._cb_local_brake)
        rospy.Subscriber(f"/{self.robot_name}/in_fuel_zone",
                         Bool,    self._cb_fuel_zone)
        rospy.Subscriber(f"/{self.robot_name}/in_charge_gate_zone",
                         Bool,    self._cb_charge_gate)
        rospy.Subscriber(f"/{self.robot_name}/in_merge_zone",
                         Bool,    self._cb_merge)
        for cam in self.camera_names:
            rospy.Subscriber(
                f"/{self.robot_name}/{cam}/in_charge_gate_zone",
                Bool,
                self._make_self_camera_cb(cam, "charge_gate"))
        rospy.Subscriber(f"/{self.robot_name}/tag_visible",
                         Bool,    self._cb_tag_visible)
        rospy.Subscriber(f"/{self.robot_name}/front_range",
                         Range,   self._cb_tof)
        rospy.Subscriber(f"/{self.robot_name}/lap_count",
                         Float32, self._cb_lap_count)
        rospy.Subscriber(f"/{self.robot_name}/set_driving_mode",
                         String,  self._cb_set_mode)
        rospy.Subscriber("/global_brake",
                         Bool,    self._cb_global_brake)

        for robot in self.other_robots:
            rospy.Subscriber(f"/{robot}/power_level", Float32,
                             self._make_power_cb(robot))
            rospy.Subscriber(f"/{robot}/in_fuel_zone", Bool,
                             self._make_fuel_cb(robot))
            rospy.Subscriber(f"/{robot}/in_charge_gate_zone", Bool,
                             self._make_charge_gate_cb(robot))
            rospy.Subscriber(f"/{robot}/in_merge_zone", Bool,
                             self._make_merge_cb(robot))
            for cam in self.camera_names:
                rospy.Subscriber(
                    f"/{robot}/{cam}/in_fuel_zone", Bool,
                    self._make_other_camera_fuel_cb(robot, cam))

        # ---- Control timer ----------------------------------------------------
        rospy.Timer(rospy.Duration(1.0 / self.control_rate), self._control_loop)

        rospy.loginfo(
            f"[VirtualDriver/{self.robot_name}] "
            f"Started in '{self.mode.value}' mode at {self.control_rate} Hz")
        self.mode_pub.publish(String(data=self.mode.value))

    # ==========================================================================
    # Callbacks
    # ==========================================================================

    def _cb_power(self, msg: Float32):
        self.power_level = msg.data

    def _cb_speed(self, msg: Float32):
        self.current_speed = msg.data
        self.speed_offset = int(round(
            (self.current_speed - SPEED_START) / SPEED_STEP))

    def _cb_local_brake(self, msg: Bool):
        self.local_brake = msg.data

    def _cb_fuel_zone(self, msg: Bool):
        self.in_fuel_zone = msg.data

    def _cb_charge_gate(self, msg: Bool):
        self.in_charge_gate = msg.data

    def _cb_merge(self, msg: Bool):
        self.in_merge_zone = msg.data

    def _make_self_camera_cb(self, camera_name: str, field: str):
        def cb(msg: Bool):
            if field == "charge_gate":
                self.in_charge_gate_by_camera[camera_name] = msg.data
        return cb

    def _cb_tag_visible(self, msg: Bool):
        self.tag_visible = msg.data

    def _cb_tof(self, msg: Range):
        now = rospy.Time.now().to_sec()
        new_range = msg.range

        if not math.isfinite(new_range) or new_range <= 0.0:
            self.tof_range = 9.9
            self.front_range_stamp = now
            self.front_closing_speed = 0.0
            self.front_ttc = float("inf")
            return

        if self.front_range_stamp is not None:
            dt = now - self.front_range_stamp
            if dt > 1.0e-3:
                instant_closing = max(0.0, (self.tof_range - new_range) / dt)
                alpha = AGGRESSIVE_CLOSING_ALPHA
                self.front_closing_speed = (
                    (1.0 - alpha) * self.front_closing_speed
                    + alpha * instant_closing
                )
                if self.front_closing_speed > 1.0e-2:
                    margin = max(0.0, new_range - self.aggressive_safe_distance)
                    self.front_ttc = margin / self.front_closing_speed
                else:
                    self.front_ttc = float("inf")

        self.tof_range = new_range
        self.front_range_stamp = now

    def _cb_lap_count(self, msg: Float32):
        new_lap_count = msg.data

        if self.last_lap_count is None:
            self.lap_count = new_lap_count
            self.last_lap_count = new_lap_count
            self.lap_start_power = self.power_level
            return

        if new_lap_count > self.last_lap_count:
            lap_delta = new_lap_count - self.last_lap_count
            power_drop = self.lap_start_power - self.power_level
            if lap_delta > 0.0 and power_drop >= AGGRESSIVE_MIN_VALID_LAP_DROP:
                per_lap_drop = power_drop / lap_delta
                if self.battery_per_lap_learned:
                    alpha = AGGRESSIVE_LAP_EMA_ALPHA
                    self.aggressive_battery_per_lap_estimate = (
                        alpha * per_lap_drop
                        + (1.0 - alpha) * self.aggressive_battery_per_lap_estimate
                    )
                else:
                    self.aggressive_battery_per_lap_estimate = per_lap_drop
                    self.battery_per_lap_learned = True

                rospy.loginfo(
                    "[VirtualDriver/%s] Aggressive learned %.1f%%/lap; "
                    "charge threshold %.1f%%",
                    self.robot_name,
                    self.aggressive_battery_per_lap_estimate,
                    self.aggressive_required_power())

            self.lap_start_power = self.power_level
        elif new_lap_count < self.last_lap_count:
            self.lap_start_power = self.power_level

        self.lap_count = new_lap_count
        self.last_lap_count = new_lap_count

    def _cb_global_brake(self, msg: Bool):
        if msg.data and not self.global_brake:
            # Game just stopped - lock local brake if it is currently released.
            if not self.local_brake:
                self.joy_pub.publish(self._joy_press(BTN_X))
                rospy.loginfo(
                    f"[VirtualDriver/{self.robot_name}] Brake locked for game stop")
            self.brake_released = False
            self.speed_offset   = 0
            self.charge_state   = ChargeState.IDLE
            self.waiting_for_fuel = False
            self.gate_release_sent = False
            self.waiting_for_merge = False
            self.merge_release_sent = False
            self.seeking_fuel = False
            self.leaving_charge = False
        self.global_brake = msg.data

    def _cb_set_mode(self, msg: String):
        try:
            new_mode = DrivingMode(msg.data.strip().lower())
        except ValueError:
            rospy.logwarn(
                f"[VirtualDriver/{self.robot_name}] "
                f"Unknown mode '{msg.data}'. "
                f"Valid: {[m.value for m in DrivingMode]}")
            return

        if new_mode != self.mode:
            self.mode           = new_mode
            self.brake_released = False   # re-release brake on mode change
            self.speed_offset   = 0
            self.charge_state   = ChargeState.IDLE
            self.waiting_for_fuel = False
            self.gate_release_sent = False
            self.waiting_for_merge = False
            self.merge_release_sent = False
            self.seeking_fuel = False
            self.leaving_charge = False
            rospy.loginfo(
                f"[VirtualDriver/{self.robot_name}] Mode -> {self.mode.value}")
            self.mode_pub.publish(String(data=self.mode.value))

    def _make_power_cb(self, name: str):
        def cb(msg: Float32):
            self.other_power[name] = msg.data
        return cb

    def _make_fuel_cb(self, name: str):
        def cb(msg: Bool):
            self.other_in_fuel[name] = msg.data
        return cb

    def _make_charge_gate_cb(self, name: str):
        def cb(msg: Bool):
            self.other_in_charge_gate[name] = msg.data
        return cb

    def _make_other_camera_fuel_cb(self, name: str, camera_name: str):
        def cb(msg: Bool):
            self.other_in_fuel_by_camera[name][camera_name] = msg.data
        return cb

    def _make_merge_cb(self, name: str):
        def cb(msg: Bool):
            self.other_in_merge[name] = msg.data
        return cb

    # ==========================================================================
    # Joy message helpers
    # ==========================================================================

    @staticmethod
    def _make_joy(buttons=None, axes=None) -> Joy:
        msg          = Joy()
        msg.header.stamp = rospy.Time.now()
        msg.buttons  = buttons if buttons is not None else [0] * 12
        msg.axes     = axes    if axes    is not None else [0.0] * 8
        return msg

    def _joy_press(self, *button_indices) -> Joy:
        """Return a Joy message with specified buttons pressed, all others 0."""
        buttons = [0] * 12
        for idx in button_indices:
            if 0 <= idx < len(buttons):
                buttons[idx] = 1
        return self._make_joy(buttons=buttons)

    def _release_brake_msg(self) -> Joy:
        """
        One-shot Joy message that releases the brake.
        Inside the fuel zone the B button must be held simultaneously
        (duckierace.py requires B+X to unlock inside the fuel zone).
        """
        if self.in_fuel_zone:
            return self._joy_press(BTN_X, BTN_B)
        return self._joy_press(BTN_X)

    def _step_speed(self, target_offset: int, buttons: list) -> list:
        """
        Add ONE L1 or R1 press toward target_offset if not already there.
        Modifies and returns buttons in-place; syncs from robot-reported speed.
        """
        self.speed_offset = int(round(
            (self.current_speed - SPEED_START) / SPEED_STEP))
        if self.speed_offset < target_offset:
            buttons[BTN_L1]   = 1
            self.speed_offset += 1
        elif self.speed_offset > target_offset:
            buttons[BTN_R1]   = 1
            self.speed_offset -= 1
        return buttons

    def _step_speed_down_to(self, target_offset: int, buttons: list) -> list:
        self.speed_offset = int(round(
            (self.current_speed - SPEED_START) / SPEED_STEP))
        if self.speed_offset > target_offset:
            buttons[BTN_R1] = 1
            self.speed_offset -= 1
        return buttons

    def _active_charge_gate_cameras(self) -> list:
        return [
            cam for cam in self.charge_camera_names
            if self.in_charge_gate_by_camera.get(cam, False)
        ]

    def _at_charge_gate(self) -> bool:
        return self.in_charge_gate or bool(self._active_charge_gate_cameras())

    def _active_charge_fuel_occupied_by_other(self) -> bool:
        for cam in self._active_charge_gate_cameras():
            for robot in self.other_robots:
                if self.other_in_fuel_by_camera[robot].get(cam, False):
                    return True
        return False

    def _merge_zone_occupied_by_other(self) -> bool:
        return any(self.other_in_merge.values())

    def aggressive_required_power(self) -> float:
        return (
            self.aggressive_battery_per_lap_estimate
            + self.aggressive_battery_reserve
        )

    def aggressive_hard_charge_threshold(self) -> float:
        return max(POWER_CRITICAL, self.aggressive_required_power())

    def aggressive_charge_start_threshold(self) -> float:
        return (
            self.aggressive_hard_charge_threshold()
            + self.aggressive_charge_start_margin
        )

    def aggressive_can_defer_charge(self) -> bool:
        return (
            self.power_level
            > self.aggressive_hard_charge_threshold()
            + self.aggressive_defer_margin
        )

    def aggressive_charge_target_power(self) -> float:
        hard_target = (
            self.aggressive_hard_charge_threshold()
            + self.aggressive_charge_target_margin
        )

        if self._aggressive_peer_charge_pressure():
            if self.power_level <= self.aggressive_hard_charge_threshold():
                return min(
                    self.aggressive_normal_charge_target,
                    max(self.aggressive_urgent_charge_target, hard_target))
            return min(
                self.aggressive_normal_charge_target,
                max(self.aggressive_pressure_charge_target, hard_target))

        return self.aggressive_normal_charge_target

    def _aggressive_peer_charge_pressure(self) -> bool:
        for robot in self.other_robots:
            if self.other_in_fuel.get(robot, False):
                return True
            if self.other_in_charge_gate.get(robot, False):
                return True
            if self.other_power.get(robot, 100.0) <= self.aggressive_peer_low_power:
                return True
        return False

    def _aggressive_peer_urgency(self, power_level: float) -> float:
        return self.aggressive_hard_charge_threshold() - power_level

    def _aggressive_peer_has_priority(self, robot: str) -> bool:
        peer_power = self.other_power.get(robot, 100.0)
        peer_urgency = self._aggressive_peer_urgency(peer_power)
        self_urgency = self._aggressive_peer_urgency(self.power_level)
        epsilon = self.aggressive_priority_epsilon

        if peer_urgency > self_urgency + epsilon:
            return True
        if self_urgency > peer_urgency + epsilon:
            return False
        if peer_power < self.power_level - epsilon:
            return True
        if self.power_level < peer_power - epsilon:
            return False
        return robot < self.robot_name

    def _aggressive_should_defer_charge(self) -> bool:
        if not self.aggressive_can_defer_charge():
            return False

        for robot in self.other_robots:
            peer_low = (
                self.other_power.get(robot, 100.0)
                <= self.aggressive_charge_start_threshold()
            )
            peer_present = (
                self.other_in_fuel.get(robot, False)
                or self.other_in_charge_gate.get(robot, False)
            )
            if (peer_low or peer_present) and self._aggressive_peer_has_priority(robot):
                return True

        return False

    def _aggressive_peer_has_gate_priority(self) -> bool:
        for robot in self.other_robots:
            if (
                    self.other_in_charge_gate.get(robot, False)
                    and self._aggressive_peer_has_priority(robot)):
                return True
        return False

    def _aggressive_front_safety_state(self) -> str:
        if self.front_range_stamp is None:
            return "clear"

        age = rospy.Time.now().to_sec() - self.front_range_stamp
        if age > AGGRESSIVE_RANGE_STALE_SEC:
            return "clear"

        if self.tof_range <= self.aggressive_emergency_distance:
            return "brake"

        if (
            self.front_closing_speed > 1.0e-2
            and self.front_ttc <= self.aggressive_ttc_brake
        ):
            return "brake"

        if (
            self.tof_range <= self.aggressive_safe_distance
            or (
                self.front_closing_speed > 1.0e-2
                and self.front_ttc <= self.aggressive_ttc_slow
            )
        ):
            return "slow"

        if self.tof_range < OVERTAKE_DIST:
            return "overtake"

        return "clear"

    def _charge_gate_control_msg(self) -> Joy:
        return self._joy_press(BTN_B)

    def _charge_gate_slow_msg(self) -> Joy:
        buttons = [0] * 12
        buttons[BTN_B] = 1
        self._step_speed_down_to(STEPS_TO_HALF, buttons)
        return self._make_joy(buttons=buttons)

    def _charge_gate_wait_msg(self) -> Joy:
        if not self.local_brake and not self.waiting_for_fuel:
            self.waiting_for_fuel = True
            return self._joy_press(BTN_X)
        self.waiting_for_fuel = True
        return self._make_joy()

    def _merge_wait_msg(self) -> Joy:
        if not self.local_brake and not self.waiting_for_merge:
            self.waiting_for_merge = True
            return self._joy_press(BTN_X)
        self.waiting_for_merge = True
        return self._make_joy()

    def _merge_release_msg(self) -> Joy:
        if self.local_brake:
            if not self.merge_release_sent:
                self.merge_release_sent = True
                return self._release_brake_msg()
            return self._make_joy()
        self.waiting_for_merge = False
        self.merge_release_sent = False
        return None

    def _charge_control_msg(
            self,
            target_power: float = POWER_CHARGE_FULL,
            wait_for_merge: bool = True) -> Joy:
        if self.charge_state == ChargeState.LOCKING:
            self.charge_state = ChargeState.CHARGING
            rospy.loginfo(f"[VirtualDriver/{self.robot_name}] Brake locked for charging")
            return self._joy_press(BTN_X)

        if self.charge_state == ChargeState.CHARGING:
            if self.power_level >= target_power:
                if wait_for_merge and self._merge_zone_occupied_by_other():
                    return self._merge_wait_msg()
                self.charge_state = ChargeState.IDLE
                self.waiting_for_merge = False
                self.merge_release_sent = False
                self.leaving_charge = True
                rospy.loginfo(f"[VirtualDriver/{self.robot_name}] Charge complete, releasing brake")
                return self._joy_press(BTN_B, BTN_X)
            return self._joy_press(BTN_Y)

        return self._make_joy()

    # ==========================================================================
    # Main control loop (called by ROS timer at self.control_rate Hz)
    # ==========================================================================

    def _control_loop(self, _event):
        # MANUAL mode: leave the physical joystick in full control
        if self.mode == DrivingMode.MANUAL:
            return

        # Game not running: do nothing (brake already engaged by race GUI)
        if self.global_brake:
            return

        # --- One-shot brake release at game start ----------------------------
        if not self.brake_released:
            if self.local_brake:
                joy_msg = self._release_brake_msg()
                self.joy_pub.publish(joy_msg)
                rospy.loginfo(
                    f"[VirtualDriver/{self.robot_name}] Brake released for '{self.mode.value}' mode")
            else:
                rospy.loginfo(
                    f"[VirtualDriver/{self.robot_name}] Brake already released for '{self.mode.value}' mode")
            self.brake_released = True
            # Give duckierace.py one cycle (debounce window) before next command
            return

        strategy = STRATEGIES.get(self.mode)
        if strategy is None:
            return

        self.joy_pub.publish(strategy.decide(self, self.actions))


# ---------------------------------------------------------------------------

if __name__ == "__main__":
    try:
        VirtualDriver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
