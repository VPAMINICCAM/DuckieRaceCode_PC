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
               switches to yellow line to overtake slow robots ahead.
COOPERATIVE  - Medium speed, yields at obstacles, shares the fuel zone fairly
               with the other robots.
ADAPTIVE     - Switches automatically between aggressive / conservative based
               on the current power level; rushes to the fuel zone when low.

ROS Interface
-------------
Subscribed topics (all absolute paths):
  /{robot_name}/power_level        Float32  Virtual power level (0-100 %)
  /{robot_name}/local_brake        Bool     Current local brake state
  /{robot_name}/in_fuel_zone       Bool     Whether robot is in the fuel zone
  /{robot_name}/in_charge_gate_zone Bool    Whether robot is at charge-gate entry
  /{robot_name}/in_merge_zone      Bool     Whether robot is in the merge zone
  /{robot_name}/front_range        Range    ToF distance to obstacle ahead
  /{robot_name}/set_driving_mode   String   Runtime mode-change command
  /{other_robot}/power_level       Float32  Peer robots' power (cooperative mode)
  /{other_robot}/in_fuel_zone      Bool     Peer robots' fuel-zone occupancy
  /{other_robot}/in_merge_zone     Bool     Peer robots' merge-zone occupancy
  /global_brake                    Bool     Race-level start / stop signal

Published topics:
  /{robot_name}/joy                Joy      Synthetic joystick commands
  /{robot_name}/driving_mode       String   Active mode name (latched)

Parameters
----------
~robot_name    str    default 'fiona'         Robot ROS namespace
~driving_mode  str    default 'conservative'  Initial behavior mode
~all_robots    list   default [fiona,lucas]
~control_rate  float  default 2.0             Control-loop frequency [Hz]
"""

import rospy
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

# Power thresholds
POWER_CRITICAL    = 15.0   # % - charge now regardless of mode
CONSERVATIVE_CHARGE_START = 55.0  # % - conservative robot starts seeking charge below this
POWER_CHARGE_FULL = 95.0   # % - stop charging above this
COOP_CHARGE_SELF  = 50.0   # % - cooperative robot charges below this
COOP_CHARGE_PEER  = 30.0   # % - cooperative robot yields if peer < this

# Obstacle distance [m]
OVERTAKE_DIST = 0.30        # activate yellow line to pass slow robot ahead
YIELD_DIST    = 0.25        # slow down (cooperative yield)


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
            else:
                return actions.hold_yellow_line()

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
                and driver._active_charge_gate_cameras()
                and not driver.in_fuel_zone):
            if (self.waits_for_fuel_occupancy(driver)
                    and driver._active_charge_fuel_occupied_by_other()):
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
            return actions.hold_yellow_line()

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
        return driver.power_level < POWER_CRITICAL

    def normal_drive(self, driver, actions: DriverActions) -> Joy:
        buttons = [0] * 12
        driver._step_speed(STEPS_TO_MAX, buttons)
        if driver.tof_range < OVERTAKE_DIST:
            buttons[BTN_B] = 1
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

        self.robot_name  = rospy.get_param("~robot_name", "fiona")
        mode_str         = rospy.get_param("~driving_mode", "conservative")
        all_robots       = rospy.get_param(
            "~all_robots", ["fiona", "lucas"])
        self.camera_names = rospy.get_param(
            "~camera_names", ["usb_cam_1", "usb_cam_2"])
        self.charge_camera_names = rospy.get_param(
            "~charge_camera_names", ["usb_cam_2"])
        self.control_rate = float(rospy.get_param("~control_rate", 2.0))

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
        self.other_power       = {r: 75.0 for r in self.other_robots}
        self.other_in_fuel     = {r: False for r in self.other_robots}
        self.other_in_merge    = {r: False for r in self.other_robots}
        self.in_charge_gate_by_camera = {cam: False for cam in self.camera_names}
        self.other_in_fuel_by_camera = {
            r: {cam: False for cam in self.camera_names}
            for r in self.other_robots
        }

        # ---- Internal driver state -------------------------------------------
        # Have we already sent the one-shot brake-release Joy message?
        self.brake_released    = False
        # Running estimate of the current speed step offset from SPEED_START.
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
        rospy.Subscriber(f"/{self.robot_name}/set_driving_mode",
                         String,  self._cb_set_mode)
        rospy.Subscriber("/global_brake",
                         Bool,    self._cb_global_brake)

        for robot in self.other_robots:
            rospy.Subscriber(f"/{robot}/power_level", Float32,
                             self._make_power_cb(robot))
            rospy.Subscriber(f"/{robot}/in_fuel_zone", Bool,
                             self._make_fuel_cb(robot))
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
        self.tof_range = msg.range

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
        Modifies and returns buttons in-place; updates self.speed_offset.
        """
        if self.speed_offset < target_offset:
            buttons[BTN_L1]   = 1
            self.speed_offset += 1
        elif self.speed_offset > target_offset:
            buttons[BTN_R1]   = 1
            self.speed_offset -= 1
        return buttons

    def _active_charge_gate_cameras(self) -> list:
        return [
            cam for cam in self.charge_camera_names
            if self.in_charge_gate_by_camera.get(cam, False)
        ]

    def _active_charge_fuel_occupied_by_other(self) -> bool:
        for cam in self._active_charge_gate_cameras():
            for robot in self.other_robots:
                if self.other_in_fuel_by_camera[robot].get(cam, False):
                    return True
        return False

    def _merge_zone_occupied_by_other(self) -> bool:
        return any(self.other_in_merge.values())

    def _charge_gate_control_msg(self) -> Joy:
        return self._joy_press(BTN_B)

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
