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
  /{robot_name}/aggressive_lap_energy Float32 Learned / conservative lap budget
  /{robot_name}/one_lap_required_power Float32 E + reserve for peer arbitration
  /{robot_name}/charge_session_count UInt32 Completed charge slots (fair tie-break)
  /{robot_name}/charge_request     Bool     Battery policy requests a charge
  /{robot_name}/charger_claim      Bool     Car is approaching, waiting, or using charger

Parameters
----------
~robot_name    str    default 'daisy'         Robot ROS namespace
~driving_mode  str    default 'conservative'  Initial behavior mode
~all_robots    list   default [lucas,daisy]
~control_rate  float  default 2.0             Control-loop frequency [Hz]
~aggressive_battery_per_lap_default float default 70.0 Conservative first-lap prior
~aggressive_battery_reserve float default 10.0  Reserve retained after a lap
~aggressive_calibration_min_power float default 80.0 First-lap charge floor
~charge_gate_hold_sec float default 1.2         Gate-event latch duration
"""

import rospy
import math
import os
import pickle
import random
from sensor_msgs.msg import Joy, Range
from std_msgs.msg import Bool, Float32, String, UInt32
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
    QLEARNING    = "qlearning"


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
QLEARNING_ALPHA   = 0.15
QLEARNING_GAMMA   = 0.90
QLEARNING_EPSILON = 0.25
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
# Start from the conservative result of the last race rather than gambling the
# first learning lap on the old 20 % prior.  A completed uncharged lap replaces
# this with the robot-specific estimate.
AGGRESSIVE_BATTERY_PER_LAP_DEFAULT = 70.0
AGGRESSIVE_BATTERY_RESERVE = 10.0
AGGRESSIVE_LAP_EMA_ALPHA = 0.35
AGGRESSIVE_MIN_VALID_LAP_DROP = 1.0
AGGRESSIVE_CHARGE_START_MARGIN = 12.0
AGGRESSIVE_PRIORITY_EPSILON = 2.0
# A robot without a clean lap model only enters the charger below this floor.
# It then charges fully so the next uncharged lap can calibrate its model.
AGGRESSIVE_CALIBRATION_MIN_POWER = 80.0
AGGRESSIVE_CALIBRATION_CHARGE_TARGET = POWER_CHARGE_FULL
# Gate pulses are much shorter than the 2 Hz control loop in normal launches.
AGGRESSIVE_CHARGE_GATE_HOLD_SEC = 1.2
# Let simultaneous gate arrivals exchange their latched claims before either
# car is admitted to the single charger.
AGGRESSIVE_GATE_ARBITRATION_SEC = 0.35
AGGRESSIVE_BRAKE_COMMAND_RETRY_SEC = 0.8
AGGRESSIVE_CHARGER_STATUS_TIMEOUT_SEC = 2.0
# A full charge is allowed only if the peer can safely continue racing.  The
# additional "one complete lap" test is applied in aggressive_charge_target.
AGGRESSIVE_FULL_CHARGE_PEER_SAFE_LAPS = 1
AGGRESSIVE_CHARGE_TARGET_MARGIN = 5.0
AGGRESSIVE_MIN_CHARGE_GAIN = 1.0


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

    def wait_at_charge_gate(self, retry_brake: bool = False) -> Joy:
        return self.driver._charge_gate_wait_msg(retry_brake=retry_brake)

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

    def uses_aggressive_charge_policy(self, driver) -> bool:
        """Whether this strategy opts into the learned shared-charger policy."""
        return False

    def normal_drive(self, driver, actions: DriverActions) -> Joy:
        return actions.idle()

    def decide(self, driver, actions: DriverActions) -> Joy:
        aggressive_policy = self.uses_aggressive_charge_policy(driver)

        if driver.charge_state != ChargeState.IDLE:
            return actions.charge(
                (driver.active_charge_target if aggressive_policy
                 else self.charge_target_power(driver)),
                self.waits_for_merge_occupancy(driver))

        # Hard emergency stop for non-aggressive modes.  Aggressive has its own
        # TTC + distance safety logic inside normal_drive; skip the check there
        # to avoid interfering with its more nuanced slow/overtake state machine.
        if not aggressive_policy and not driver.local_brake:
            if (driver.front_range_stamp is not None
                    and rospy.Time.now().to_sec() - driver.front_range_stamp
                    < AGGRESSIVE_RANGE_STALE_SEC
                    and driver.tof_range < AGGRESSIVE_EMERGENCY_DISTANCE):
                return actions.press(BTN_X)

        # This persistent handoff state is specific to aggressive mode.  The
        # other strategies retain their existing simple charge-zone behavior.
        if aggressive_policy and driver.waiting_for_fuel:
            if driver.in_fuel_zone:
                driver.waiting_for_fuel = False
                driver.seeking_fuel = False
                driver._clear_charge_gate_arbitration()
            elif driver._charge_gate_arbitrating():
                return actions.wait_at_charge_gate(retry_brake=True)
            elif (self.waits_for_fuel_occupancy(driver)
                  and (driver._charger_occupied_by_other()
                       or self.waits_for_charge_gate_priority(driver))):
                return actions.wait_at_charge_gate(retry_brake=True)
            else:
                driver.waiting_for_fuel = False
                driver.seeking_fuel = True
                driver._clear_charge_gate_arbitration()
                if driver.local_brake:
                    return driver._release_gate_brake_if_due(actions)
                return actions.slow_for_charge_gate()

        if driver.seeking_fuel:
            if driver.in_fuel_zone:
                driver.seeking_fuel = False
                if aggressive_policy:
                    driver._clear_charge_gate_arbitration()
            elif (self.waits_for_fuel_occupancy(driver)
                  and (driver._charger_occupied_by_other()
                       if aggressive_policy
                       else driver._active_charge_fuel_occupied_by_other())):
                driver.seeking_fuel = False
                if aggressive_policy:
                    driver.waiting_for_fuel = True
                    driver._reset_gate_release_retry()
                    return actions.wait_at_charge_gate(retry_brake=True)
                driver.gate_release_sent = False
                return actions.wait_at_charge_gate()
            elif self.defers_charge_for_peer(driver):
                driver.seeking_fuel = False
                if aggressive_policy:
                    driver._reset_gate_release_retry()
                else:
                    driver.gate_release_sent = False
                return self.normal_drive(driver, actions)
            elif aggressive_policy and driver.local_brake:
                return driver._release_gate_brake_if_due(actions)
            else:
                return actions.slow_for_charge_gate()

        # A completed charging slot must exit the fuel area before evaluating
        # demand again; a one-lap target can intentionally remain below the
        # early-start threshold while another car is waiting.
        if aggressive_policy and driver.leaving_charge:
            if driver.in_merge_zone:
                driver.leaving_charge = False
            else:
                return actions.hold_yellow_line()

        if driver.in_fuel_zone and self.wants_charge(driver):
            if aggressive_policy:
                driver._begin_charge(self.charge_target_power(driver))
            else:
                driver.charge_state = ChargeState.LOCKING
            return actions.charge(
                (driver.active_charge_target if aggressive_policy
                 else self.charge_target_power(driver)),
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

        if not aggressive_policy and driver.leaving_charge:
            if driver.in_merge_zone:
                driver.leaving_charge = False
            else:
                return actions.hold_yellow_line()

        # Use the latched version for all modes: at 2 Hz a brief gate detection
        # can expire before the next control tick, causing the robot to miss its
        # window to enter seeking_fuel.
        at_charge_gate = driver._at_charge_gate()
        if (self.wants_charge(driver)
                and at_charge_gate
                and not driver.in_fuel_zone):
            if aggressive_policy:
                if not driver._charge_gate_arbitrating():
                    driver._start_charge_gate_arbitration()
                    driver.waiting_for_fuel = True
                    driver._reset_gate_release_retry()
                    return actions.wait_at_charge_gate(retry_brake=True)
                if (self.waits_for_fuel_occupancy(driver)
                        and (driver._charger_occupied_by_other()
                             or self.waits_for_charge_gate_priority(driver))):
                    driver.waiting_for_fuel = True
                    driver._reset_gate_release_retry()
                    return actions.wait_at_charge_gate(retry_brake=True)
                # Persist admission before toggling the brake.  This keeps the
                # car on the yellow line if the gate pulse expires meanwhile.
                driver.seeking_fuel = True
                driver.waiting_for_fuel = False
                driver._clear_charge_gate_arbitration()
                if driver.local_brake:
                    return driver._release_gate_brake_if_due(actions)
                driver._reset_gate_release_retry()
                return actions.slow_for_charge_gate()

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
        if aggressive_policy:
            driver._clear_charge_gate_arbitration()
            driver._reset_gate_release_retry()
        else:
            driver.gate_release_sent = False
        return self.normal_drive(driver, actions)


class ConservativeStrategy(DrivingStrategyBase):
    def wants_charge(self, driver) -> bool:
        return driver.power_level < CONSERVATIVE_CHARGE_START

    def normal_drive(self, driver, actions: DriverActions) -> Joy:
        return actions.speed_step_msg(STEPS_TO_MIN)


class AggressiveStrategy(DrivingStrategyBase):
    def uses_aggressive_charge_policy(self, driver) -> bool:
        return driver.mode == DrivingMode.AGGRESSIVE

    def wants_charge(self, driver) -> bool:
        if not driver.aggressive_charge_demand():
            return False
        if driver.power_level < POWER_CRITICAL:
            return True
        if not driver.battery_per_lap_learned:
            return True
        if driver.power_level <= driver.aggressive_hard_charge_threshold():
            return True
        return not driver._aggressive_should_defer_charge()

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


class QLearningStrategy(DrivingStrategyBase):
    ACTIONS = ["conservative", "aggressive", "charge"]

    def __init__(self):
        self.q_table = {}
        self.last_state = {}
        self.last_action = {}
        self.chosen_action = {}

    def _discrete_state(self, driver):
        power = 0 if driver.power_level < 30.0 else 1 if driver.power_level < 70.0 else 2
        obstacle = 1 if driver.tof_range < OVERTAKE_DIST else 0
        in_fuel = 1 if driver.in_fuel_zone else 0
        in_merge = 1 if driver.in_merge_zone else 0
        return (power, obstacle, in_fuel, in_merge)

    def _get_q(self, state, action):
        return self.q_table.get((state, action), 0.0)

    def _best_future_q(self, state):
        return max(self._get_q(state, action) for action in self.ACTIONS)

    def _choose_action(self, driver, state):
        if random.random() < QLEARNING_EPSILON:
            return random.choice(self.ACTIONS)
        return max(
            self.ACTIONS,
            key=lambda action: self._get_q(state, action)
        )

    def _reward(self, driver, action):
        reward = 0.0
        reward += (driver.power_level - 50.0) / 50.0
        reward += max(-1.0, min(1.0, driver.speed_offset / float(STEPS_TO_MAX)))
        if driver.power_level < POWER_CRITICAL:
            reward -= 1.5
        if action == "charge" and driver.power_level < 70.0:
            reward += 0.8
        if action == "aggressive" and driver.power_level > 40.0:
            reward += 0.2
        if action == "conservative" and driver.power_level < 40.0:
            reward += 0.2
        if driver.in_fuel_zone and driver.charge_state == ChargeState.CHARGING:
            reward += 1.0
        if driver.in_merge_zone and driver.local_brake:
            reward -= 0.3
        return reward

    def _update_q(self, driver, prev_state, prev_action, next_state, reward):
        old_value = self._get_q(prev_state, prev_action)
        future_best = self._best_future_q(next_state)
        self.q_table[(prev_state, prev_action)] = (
            old_value + QLEARNING_ALPHA *
            (reward + QLEARNING_GAMMA * future_best - old_value)
        )

    def wants_charge(self, driver) -> bool:
        # Action is chosen once per timestep in decide(); this just reads it.
        # Calling _choose_action() here would re-roll epsilon-greedy on every
        # call and corrupt last_action used by the Q-update next timestep.
        return self.chosen_action.get(driver.robot_name, "conservative") == "charge"

    def normal_drive(self, driver, actions: DriverActions) -> Joy:
        action = self.chosen_action.get(driver.robot_name, "conservative")
        if action == "aggressive":
            return AggressiveStrategy().normal_drive(driver, actions)
        return ConservativeStrategy().normal_drive(driver, actions)

    def decide(self, driver, actions: DriverActions) -> Joy:
        name = driver.robot_name
        state = self._discrete_state(driver)
        if name in self.last_state and name in self.last_action:
            reward = self._reward(driver, self.last_action[name])
            self._update_q(
                driver,
                self.last_state[name],
                self.last_action[name],
                state,
                reward)
        # Choose the action exactly once per timestep before base class calls
        # wants_charge() or normal_drive().
        action = self._choose_action(driver, state)
        self.chosen_action[name] = action
        self.last_state[name] = state
        self.last_action[name] = action
        return super().decide(driver, actions)

    def save(self, path: str):
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, "wb") as f:
                pickle.dump(self.q_table, f)
            rospy.loginfo("[QLearning] Q-table saved to %s (%d entries)", path, len(self.q_table))
        except Exception as e:
            rospy.logwarn("[QLearning] Could not save Q-table: %s", e)

    def load(self, path: str):
        try:
            with open(path, "rb") as f:
                self.q_table = pickle.load(f)
            rospy.loginfo("[QLearning] Q-table loaded from %s (%d entries)", path, len(self.q_table))
        except FileNotFoundError:
            rospy.loginfo("[QLearning] No saved Q-table at %s, starting fresh", path)
        except Exception as e:
            rospy.logwarn("[QLearning] Could not load Q-table: %s", e)


STRATEGIES = {
    DrivingMode.CONSERVATIVE: ConservativeStrategy(),
    DrivingMode.AGGRESSIVE: AggressiveStrategy(),
    DrivingMode.COOPERATIVE: CooperativeStrategy(),
    DrivingMode.ADAPTIVE: AdaptiveStrategy(),
    DrivingMode.QLEARNING: QLearningStrategy(),
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
        self.aggressive_priority_epsilon = float(rospy.get_param(
            "~aggressive_priority_epsilon",
            AGGRESSIVE_PRIORITY_EPSILON))
        self.aggressive_calibration_min_power = float(rospy.get_param(
            "~aggressive_calibration_min_power",
            AGGRESSIVE_CALIBRATION_MIN_POWER))
        self.aggressive_calibration_charge_target = float(rospy.get_param(
            "~aggressive_calibration_charge_target",
            AGGRESSIVE_CALIBRATION_CHARGE_TARGET))
        self.charge_gate_hold_sec = float(rospy.get_param(
            "~charge_gate_hold_sec", AGGRESSIVE_CHARGE_GATE_HOLD_SEC))
        self.charge_gate_arbitration_sec = float(rospy.get_param(
            "~charge_gate_arbitration_sec", AGGRESSIVE_GATE_ARBITRATION_SEC))
        self.brake_command_retry_sec = float(rospy.get_param(
            "~brake_command_retry_sec", AGGRESSIVE_BRAKE_COMMAND_RETRY_SEC))
        self.charger_status_timeout_sec = float(rospy.get_param(
            "~charger_status_timeout_sec", AGGRESSIVE_CHARGER_STATUS_TIMEOUT_SEC))
        self.aggressive_full_charge_peer_safe_laps = max(1, int(rospy.get_param(
            "~aggressive_full_charge_peer_safe_laps",
            AGGRESSIVE_FULL_CHARGE_PEER_SAFE_LAPS)))
        self.aggressive_charge_target_margin = float(rospy.get_param(
            "~aggressive_charge_target_margin",
            AGGRESSIVE_CHARGE_TARGET_MARGIN))
        self.aggressive_min_charge_gain = float(rospy.get_param(
            "~aggressive_min_charge_gain", AGGRESSIVE_MIN_CHARGE_GAIN))
        self.aggressive_default_lap_energy = (
            self.aggressive_battery_per_lap_estimate)

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
        self.charged_since_lap_start = False
        self.charge_gate_until = 0.0
        self.charge_gate_arbitration_until = 0.0
        # Treat an unseen peer as safe until its latched state arrives.  This
        # avoids an offline/late subscriber permanently forcing short charges.
        self.other_power       = {r: 100.0 for r in self.other_robots}
        self.other_lap_energy = {
            r: self.aggressive_default_lap_energy for r in self.other_robots
        }
        self.other_one_lap_required_power = {
            r: min(100.0, self.aggressive_default_lap_energy
                   + self.aggressive_battery_reserve)
            for r in self.other_robots
        }
        self.other_charge_sessions = {r: 0 for r in self.other_robots}
        self.other_charge_requested = {r: False for r in self.other_robots}
        self.other_charger_claimed = {r: False for r in self.other_robots}
        self.other_charge_request_stamp = {r: None for r in self.other_robots}
        self.other_charger_claim_stamp = {r: None for r in self.other_robots}
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
        self.active_charge_target = POWER_CHARGE_FULL
        self.active_charge_target_reason = ""
        self.charge_session_count = 0
        self.charge_lock_last_sent = None
        self.waiting_for_fuel  = False
        self.gate_release_sent = False
        self.gate_release_last_sent = None
        self.fuel_wait_brake_last_sent = None
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
        # These latched topics let each driver compare the peer's *own* learned
        # energy model rather than applying its local model to both vehicles.
        self.lap_energy_pub = rospy.Publisher(
            f"/{self.robot_name}/aggressive_lap_energy", Float32,
            queue_size=1, latch=True)
        self.one_lap_required_power_pub = rospy.Publisher(
            f"/{self.robot_name}/one_lap_required_power", Float32,
            queue_size=1, latch=True)
        self.charge_sessions_pub = rospy.Publisher(
            f"/{self.robot_name}/charge_session_count", UInt32,
            queue_size=1, latch=True)
        self.charge_request_pub = rospy.Publisher(
            f"/{self.robot_name}/charge_request", Bool, queue_size=1, latch=True)
        self.charger_claim_pub = rospy.Publisher(
            f"/{self.robot_name}/charger_claim", Bool, queue_size=1, latch=True)

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
        rospy.Subscriber("/reset_laps", Bool, self._cb_reset_laps)

        for robot in self.other_robots:
            rospy.Subscriber(f"/{robot}/power_level", Float32,
                             self._make_power_cb(robot))
            rospy.Subscriber(f"/{robot}/in_fuel_zone", Bool,
                             self._make_fuel_cb(robot))
            rospy.Subscriber(f"/{robot}/in_charge_gate_zone", Bool,
                             self._make_charge_gate_cb(robot))
            rospy.Subscriber(f"/{robot}/in_merge_zone", Bool,
                             self._make_merge_cb(robot))
            rospy.Subscriber(f"/{robot}/aggressive_lap_energy", Float32,
                             self._make_lap_energy_cb(robot))
            rospy.Subscriber(f"/{robot}/one_lap_required_power", Float32,
                             self._make_required_power_cb(robot))
            rospy.Subscriber(f"/{robot}/charge_session_count", UInt32,
                             self._make_charge_sessions_cb(robot))
            rospy.Subscriber(f"/{robot}/charge_request", Bool,
                             self._make_charge_request_cb(robot))
            rospy.Subscriber(f"/{robot}/charger_claim", Bool,
                             self._make_charger_claim_cb(robot))
            for cam in self.camera_names:
                rospy.Subscriber(
                    f"/{robot}/{cam}/in_fuel_zone", Bool,
                    self._make_other_camera_fuel_cb(robot, cam))

        # ---- Q-learning persistence ------------------------------------------
        if self.mode == DrivingMode.QLEARNING:
            self._q_table_path = os.path.expanduser(
                f"~/.ros/q_table_{self.robot_name}.pkl")
            STRATEGIES[DrivingMode.QLEARNING].load(self._q_table_path)
            rospy.on_shutdown(self._save_q_table)

        # ---- Control timer ----------------------------------------------------
        rospy.Timer(rospy.Duration(1.0 / self.control_rate), self._control_loop)

        rospy.loginfo(
            f"[VirtualDriver/{self.robot_name}] "
            f"Started in '{self.mode.value}' mode at {self.control_rate} Hz")
        self.mode_pub.publish(String(data=self.mode.value))
        self._publish_aggressive_budget()
        self._publish_charge_status()

    def _save_q_table(self):
        STRATEGIES[DrivingMode.QLEARNING].save(self._q_table_path)

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
        if msg.data:
            self._latch_charge_gate()

    def _cb_merge(self, msg: Bool):
        self.in_merge_zone = msg.data

    def _make_self_camera_cb(self, camera_name: str, field: str):
        def cb(msg: Bool):
            if field == "charge_gate":
                self.in_charge_gate_by_camera[camera_name] = msg.data
                if msg.data:
                    self._latch_charge_gate()
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
        # The lap-energy model is meaningful only for the aggressive driving
        # profile; conservative/cooperative/adaptive modes must not tune it.
        if self.mode != DrivingMode.AGGRESSIVE:
            return

        new_lap_count = msg.data

        if self.last_lap_count is None:
            self.lap_count = new_lap_count
            self.last_lap_count = new_lap_count
            self.lap_start_power = self.power_level
            return

        if new_lap_count > self.last_lap_count:
            lap_delta = new_lap_count - self.last_lap_count
            power_drop = self.lap_start_power - self.power_level
            if self.charged_since_lap_start:
                rospy.loginfo(
                    "[VirtualDriver/%s] Ignoring lap energy sample containing a charge",
                    self.robot_name)
            elif lap_delta > 0.0 and power_drop >= AGGRESSIVE_MIN_VALID_LAP_DROP:
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
                self._publish_aggressive_budget()

            self.lap_start_power = self.power_level
            self.charged_since_lap_start = False
        elif new_lap_count < self.last_lap_count:
            self.lap_start_power = self.power_level
            self.charged_since_lap_start = False

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
            self.active_charge_target = POWER_CHARGE_FULL
            self.active_charge_target_reason = ""
            self.charge_lock_last_sent = None
            self.waiting_for_fuel = False
            self._reset_gate_release_retry()
            self.waiting_for_merge = False
            self.merge_release_sent = False
            self.seeking_fuel = False
            self.leaving_charge = False
            self.charge_gate_until = 0.0
            self.charge_gate_arbitration_until = 0.0
        elif (
                not msg.data
                and self.global_brake
                and self.mode == DrivingMode.AGGRESSIVE):
            # The GUI resets power independently of /reset_laps.  Race start
            # is therefore the reliable point to establish a fresh baseline.
            self._reset_aggressive_learning()
        self.global_brake = msg.data
        self._publish_charge_status()

    def _cb_reset_laps(self, msg: Bool):
        if not msg.data or self.mode != DrivingMode.AGGRESSIVE:
            return
        self._reset_aggressive_learning()

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
            self.active_charge_target = POWER_CHARGE_FULL
            self.active_charge_target_reason = ""
            self.charge_lock_last_sent = None
            self.waiting_for_fuel = False
            self._reset_gate_release_retry()
            self.waiting_for_merge = False
            self.merge_release_sent = False
            self.seeking_fuel = False
            self.leaving_charge = False
            rospy.loginfo(
                f"[VirtualDriver/{self.robot_name}] Mode -> {self.mode.value}")
            self.mode_pub.publish(String(data=self.mode.value))
            if self.mode == DrivingMode.AGGRESSIVE:
                # Entering aggressive mode starts a fresh calibration interval;
                # samples collected under another driving profile are invalid.
                self._reset_aggressive_learning()
            else:
                self._publish_charge_status()

    def _make_power_cb(self, name: str):
        def cb(msg: Float32):
            self.other_power[name] = msg.data
        return cb

    def _make_lap_energy_cb(self, name: str):
        def cb(msg: Float32):
            if msg.data > 0.0:
                self.other_lap_energy[name] = msg.data
        return cb

    def _make_required_power_cb(self, name: str):
        def cb(msg: Float32):
            if msg.data > 0.0:
                self.other_one_lap_required_power[name] = min(100.0, msg.data)
        return cb

    def _make_charge_sessions_cb(self, name: str):
        def cb(msg: UInt32):
            self.other_charge_sessions[name] = int(msg.data)
        return cb

    def _make_charge_request_cb(self, name: str):
        def cb(msg: Bool):
            self.other_charge_requested[name] = bool(msg.data)
            self.other_charge_request_stamp[name] = rospy.Time.now().to_sec()
        return cb

    def _make_charger_claim_cb(self, name: str):
        def cb(msg: Bool):
            self.other_charger_claimed[name] = bool(msg.data)
            self.other_charger_claim_stamp[name] = rospy.Time.now().to_sec()
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

    def _latch_charge_gate(self):
        self.charge_gate_until = max(
            self.charge_gate_until,
            rospy.Time.now().to_sec() + self.charge_gate_hold_sec,
        )

    def _start_charge_gate_arbitration(self):
        self.charge_gate_arbitration_until = max(
            self.charge_gate_arbitration_until,
            rospy.Time.now().to_sec() + self.charge_gate_arbitration_sec,
        )

    def _charge_gate_arbitrating(self) -> bool:
        return self.charge_gate_arbitration_until > rospy.Time.now().to_sec()

    def _clear_charge_gate_arbitration(self):
        self.charge_gate_arbitration_until = 0.0

    def _at_charge_gate(self) -> bool:
        return (
            self.in_charge_gate
            or bool(self._active_charge_gate_cameras())
            or self.charge_gate_until > rospy.Time.now().to_sec()
        )

    def _raw_at_charge_gate(self) -> bool:
        """Direct zone state for non-aggressive strategies."""
        return self.in_charge_gate or bool(self._active_charge_gate_cameras())

    def _active_charge_fuel_occupied_by_other(self) -> bool:
        for cam in self._active_charge_gate_cameras():
            for robot in self.other_robots:
                if self.other_in_fuel_by_camera[robot].get(cam, False):
                    return True
        return False

    def _charger_occupied_by_other(self) -> bool:
        """Return true while another robot owns the single physical charger."""
        return (
            self._active_charge_fuel_occupied_by_other()
            or any(self.other_in_fuel.values())
        )

    def _reset_gate_release_retry(self):
        self.gate_release_sent = False
        self.gate_release_last_sent = None
        self.fuel_wait_brake_last_sent = None

    def _release_gate_brake_if_due(self, actions: DriverActions) -> Joy:
        """Retry an admission brake release until local_brake confirms it."""
        now = rospy.Time.now().to_sec()
        if (
                self.gate_release_last_sent is None
                or now - self.gate_release_last_sent >= self.brake_command_retry_sec):
            self.gate_release_last_sent = now
            self.gate_release_sent = True
            return actions.release_brake()
        return actions.idle()

    def _merge_zone_occupied_by_other(self) -> bool:
        return any(self.other_in_merge.values())

    def _begin_charge(self, target_power: float):
        self.charge_state = ChargeState.LOCKING
        self.active_charge_target = min(
            POWER_CHARGE_FULL, max(0.0, float(target_power)))
        self.active_charge_target_reason = (
            "aggressive" if self.mode == DrivingMode.AGGRESSIVE else self.mode.value)
        self.charge_lock_last_sent = None
        # A lap that spans any charge is not a clean energy-consumption sample.
        self.charged_since_lap_start = True
        rospy.loginfo(
            "[VirtualDriver/%s] Charging plan: %.1f%% target",
            self.robot_name,
            self.active_charge_target,
        )

    def _publish_aggressive_budget(self):
        self.lap_energy_pub.publish(Float32(
            data=self.aggressive_battery_per_lap_estimate))
        self.one_lap_required_power_pub.publish(Float32(
            data=self.aggressive_required_power()))
        self.charge_sessions_pub.publish(UInt32(data=self.charge_session_count))

    def _publish_charge_status(self):
        aggressive = (
            self.mode == DrivingMode.AGGRESSIVE and not self.global_brake)
        requested = aggressive and self.aggressive_charge_demand()
        claimed = aggressive and (
            self.seeking_fuel
            or self.waiting_for_fuel
            or self.in_fuel_zone
            or self.charge_state != ChargeState.IDLE
            or (requested and self._at_charge_gate())
        )
        self.charge_request_pub.publish(Bool(data=requested))
        self.charger_claim_pub.publish(Bool(data=claimed))

    def _reset_aggressive_learning(self):
        self.aggressive_battery_per_lap_estimate = (
            self.aggressive_default_lap_energy)
        self.battery_per_lap_learned = False
        self.lap_count = 0.0
        self.last_lap_count = None
        self.lap_start_power = self.power_level
        self.charged_since_lap_start = False
        self.charge_session_count = 0
        self.charge_gate_until = 0.0
        self.charge_gate_arbitration_until = 0.0
        self._publish_aggressive_budget()
        self._publish_charge_status()
        rospy.loginfo(
            "[VirtualDriver/%s] Reset aggressive lap-energy calibration",
            self.robot_name)

    def aggressive_required_power(self) -> float:
        return min(100.0, (
            self.aggressive_battery_per_lap_estimate
            + self.aggressive_battery_reserve
        ))

    def aggressive_one_lap_target_power(self) -> float:
        return min(
            POWER_CHARGE_FULL,
            self.aggressive_required_power() + self.aggressive_charge_target_margin,
        )

    def aggressive_safe_laps(self) -> int:
        return self._safe_laps(
            self.power_level,
            self.aggressive_required_power(),
            self.aggressive_battery_per_lap_estimate,
        )

    @staticmethod
    def _safe_laps(power_level: float, required_power: float,
                   lap_energy: float) -> int:
        """Completed laps available while retaining the configured reserve."""
        if lap_energy <= 0.0 or power_level < required_power:
            return 0
        return 1 + int((power_level - required_power) // lap_energy)

    def _aggressive_peer_required_power(self, robot: str) -> float:
        return self.other_one_lap_required_power.get(robot, 100.0)

    def _aggressive_peer_safe_laps(self, robot: str) -> int:
        return self._safe_laps(
            self.other_power.get(robot, 100.0),
            self._aggressive_peer_required_power(robot),
            self.other_lap_energy.get(robot, self.aggressive_default_lap_energy),
        )

    def _charger_status_is_fresh(self, stamp) -> bool:
        return (
            stamp is None
            or rospy.Time.now().to_sec() - stamp <= self.charger_status_timeout_sec
        )

    def _aggressive_peer_is_contending(self, robot: str) -> bool:
        return (
            (
                self.other_charger_claimed.get(robot, False)
                and self._charger_status_is_fresh(
                    self.other_charger_claim_stamp.get(robot))
            )
            or self.other_in_fuel.get(robot, False)
            or self.other_in_charge_gate.get(robot, False)
        )

    def _aggressive_peer_has_charge_demand(self, robot: str) -> bool:
        if (
                self.other_charge_requested.get(robot, False)
                and self._charger_status_is_fresh(
                    self.other_charge_request_stamp.get(robot))):
            return True
        return (
            self._aggressive_peer_is_contending(robot)
            and self.other_power.get(robot, 100.0)
            <= self._aggressive_peer_required_power(robot)
            + self.aggressive_charge_start_margin
        )

    def _aggressive_peer_needs_charger(self) -> bool:
        return any(
            self._aggressive_peer_has_charge_demand(robot)
            for robot in self.other_robots
        )

    def _aggressive_peer_requires_handoff(self) -> bool:
        return any(
            self._aggressive_peer_has_charge_demand(robot)
            and self._aggressive_peer_safe_laps(robot)
            < self.aggressive_full_charge_peer_safe_laps
            for robot in self.other_robots
        )

    def aggressive_charge_demand(self) -> bool:
        """Demand before arbitration; unlike wants_charge it never yields."""
        if not self.battery_per_lap_learned:
            below_entry_threshold = (
                self.power_level <= self.aggressive_calibration_min_power)
        else:
            below_entry_threshold = (
                self.power_level <= self.aggressive_charge_start_threshold())
        if not below_entry_threshold:
            return False

        # Do not take a charger slot merely because we are inside the early
        # start band when the selected one-lap target is already below us.
        return (
            self.aggressive_charge_target_power()
            > self.power_level + self.aggressive_min_charge_gain
        )

    def aggressive_hard_charge_threshold(self) -> float:
        return max(POWER_CRITICAL, self.aggressive_required_power())

    def aggressive_charge_start_threshold(self) -> float:
        return (
            self.aggressive_hard_charge_threshold()
            + self.aggressive_charge_start_margin
        )

    def aggressive_can_defer_charge(self) -> bool:
        return self.aggressive_safe_laps() >= 1

    def aggressive_charge_target_power(self) -> float:
        if not self.battery_per_lap_learned:
            if (
                    self._aggressive_charger_is_contended()
                    or self._aggressive_peer_needs_charger()):
                return self.aggressive_one_lap_target_power()
            return min(POWER_CHARGE_FULL,
                       self.aggressive_calibration_charge_target)

        one_lap_target = self.aggressive_one_lap_target_power()
        if self._aggressive_charger_is_contended():
            return one_lap_target

        # A full charge must buy at least one additional complete lap.  It is
        # otherwise pure waiting time and creates an avoidable charger stop.
        full_charge_buys_lap = (
            POWER_CHARGE_FULL
            >= one_lap_target + self.aggressive_battery_per_lap_estimate
        )
        peers_can_continue = all(
            self._aggressive_peer_safe_laps(robot)
            >= self.aggressive_full_charge_peer_safe_laps
            for robot in self.other_robots
        )
        if full_charge_buys_lap and peers_can_continue:
            return POWER_CHARGE_FULL
        return one_lap_target

    def _aggressive_charger_is_contended(self) -> bool:
        return any(
            self._aggressive_peer_is_contending(robot)
            and self._aggressive_peer_has_charge_demand(robot)
            for robot in self.other_robots
        )

    def _aggressive_peer_has_priority(self, robot: str) -> bool:
        peer_power = self.other_power.get(robot, 100.0)
        peer_margin = peer_power - self._aggressive_peer_required_power(robot)
        self_margin = self.power_level - self.aggressive_required_power()
        epsilon = self.aggressive_priority_epsilon

        if peer_margin < self_margin - epsilon:
            return True
        if self_margin < peer_margin - epsilon:
            return False
        peer_sessions = self.other_charge_sessions.get(robot, 0)
        if peer_sessions < self.charge_session_count:
            return True
        if self.charge_session_count < peer_sessions:
            return False
        return robot < self.robot_name

    def _aggressive_should_defer_charge(self) -> bool:
        if not self.aggressive_can_defer_charge():
            return False

        for robot in self.other_robots:
            if (
                    self._aggressive_peer_is_contending(robot)
                    and self._aggressive_peer_has_charge_demand(robot)
                    and self._aggressive_peer_has_priority(robot)):
                return True

        return False

    def _aggressive_peer_has_gate_priority(self) -> bool:
        for robot in self.other_robots:
            if (
                    self._aggressive_peer_is_contending(robot)
                    and self._aggressive_peer_has_charge_demand(robot)
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

    def _charge_gate_wait_msg(self, retry_brake: bool = False) -> Joy:
        if not retry_brake:
            if not self.local_brake and not self.waiting_for_fuel:
                self.waiting_for_fuel = True
                return self._joy_press(BTN_X)
            self.waiting_for_fuel = True
            return self._make_joy()

        if not self.local_brake:
            now = rospy.Time.now().to_sec()
            if (
                    self.fuel_wait_brake_last_sent is None
                    or now - self.fuel_wait_brake_last_sent >= self.brake_command_retry_sec):
                self.fuel_wait_brake_last_sent = now
                self.waiting_for_fuel = True
                return self._joy_press(BTN_X)
        self.waiting_for_fuel = True
        if self.local_brake:
            self.fuel_wait_brake_last_sent = None
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
            if self.local_brake:
                self.charge_state = ChargeState.CHARGING
                self.charge_lock_last_sent = None
                rospy.loginfo(
                    f"[VirtualDriver/{self.robot_name}] Brake locked for charging")
                return self._joy_press(BTN_Y)

            now = rospy.Time.now().to_sec()
            if (
                    self.charge_lock_last_sent is None
                    or now - self.charge_lock_last_sent >= self.brake_command_retry_sec):
                self.charge_lock_last_sent = now
                return self._joy_press(BTN_X)
            return self._make_joy()

        if self.charge_state == ChargeState.CHARGING:
            # A full plan is fixed at admission so a transiently-clear peer
            # cannot expand a short shared slot.  It may only shrink if a peer
            # becomes an active claimant while we are charging.
            if self.mode == DrivingMode.AGGRESSIVE:
                one_lap_target = self.aggressive_one_lap_target_power()
                if (
                        (
                            self._aggressive_charger_is_contended()
                            or self._aggressive_peer_requires_handoff()
                        )
                        and self.active_charge_target > one_lap_target):
                    self.active_charge_target = one_lap_target
                    target_power = one_lap_target
                    rospy.loginfo(
                        "[VirtualDriver/%s] Shortened charge plan to %.1f%% for peer",
                        self.robot_name,
                        target_power,
                    )
                else:
                    target_power = self.active_charge_target

            if self.power_level >= target_power:
                if wait_for_merge and self._merge_zone_occupied_by_other():
                    return self._merge_wait_msg()
                self.charge_state = ChargeState.IDLE
                if self.mode == DrivingMode.AGGRESSIVE:
                    self.charge_session_count += 1
                    self._publish_aggressive_budget()
                # Start the next energy sample after the charge.  The lap that
                # led into this stop is intentionally discarded; the following
                # uncharged lap is the first valid calibration sample.
                self.lap_start_power = self.power_level
                self.charged_since_lap_start = False
                self.waiting_for_merge = False
                self.merge_release_sent = False
                self.leaving_charge = True
                self.active_charge_target = POWER_CHARGE_FULL
                self.active_charge_target_reason = ""
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

        self._publish_charge_status()
        self.joy_pub.publish(strategy.decide(self, self.actions))
        self._publish_charge_status()


# ---------------------------------------------------------------------------

if __name__ == "__main__":
    try:
        VirtualDriver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
