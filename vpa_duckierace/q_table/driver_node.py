"""
Q-Table Race Driver
===================
Charging WHEN decision  → Q-table (learns from experience)
Charging HOW mechanics  → same state machine as virtual driver
Speed control           → camera-based (unchanged)

The Q-table is saved to ~/.ros/duckierace_q_table.json after every lap
and loaded on startup, so learning accumulates across sessions.

Reward shaping:
  +10   per lap completed
  +bat  battery % recovered after a successful charge
  -5    per collision / hard-stop event
  -0.5  per control tick when battery < critical (urgency)
"""

from __future__ import annotations

import json
import math
import os
import sys
from enum import Enum

import numpy as np
import rospy
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Joy, Range
from std_msgs.msg import Bool, Float32, String

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PKG_DIR not in sys.path:
    sys.path.insert(0, PKG_DIR)

from q_table.q_agent import (ACTION_CHARGE, ACTION_DRIVE, ZONE_FUEL,
                              ZONE_GATE, ZONE_MERGE, ZONE_TRACK, QAgent)

# ── button indices (physical robot and virtual-driver contract) ─────────
BTN_X  = 0   # brake toggle
BTN_B  = 2   # yellow-line follow / fuel-zone brake unlock
BTN_Y  = 3   # enable charging
BTN_L1 = 4   # speed step up
BTN_R1 = 5   # speed step down

# ── charging state machine ───────────────────────────────────────────────────
class ChargeState(Enum):
    IDLE     = "idle"
    LOCKING  = "locking"
    CHARGING = "charging"

BRAKE_CONFIRM_TICKS = 10
MAX_BRAKE_RETRIES   = 2
SAVE_EVERY_N_LAPS   = 1    # save Q-table after each lap


class QDriver:
    def __init__(self):
        rospy.init_node("q_driver")

        # ── parameters ────────────────────────────────────────────────────
        self.robot       = rospy.get_param("~robot_name",          "daisy")
        self.all_robots  = list(rospy.get_param("~all_robots",     [self.robot]))
        self.rate_hz     = float(rospy.get_param("~control_rate",  5.0))
        self.cruise      = float(rospy.get_param("~cruise_speed",  0.31))
        self.min_speed   = float(rospy.get_param("~min_speed",     0.15))
        self.max_speed   = float(rospy.get_param("~max_speed",     0.35))
        self.start_speed = float(rospy.get_param("~start_speed",   0.25))
        self.speed_step  = float(rospy.get_param("~speed_step",    0.02))
        self.seek_speed  = float(rospy.get_param("~charge_seek_speed", 0.22))

        self.charge_target  = float(rospy.get_param("~charge_target_power", 92.0))
        self.critical_power = float(rospy.get_param("~critical_power",      12.0))
        self.hard_stop_m    = float(rospy.get_param("~hard_stop_range_m",   0.30))
        self.stop_dist      = float(rospy.get_param("~safety_stop_distance_m", 0.45))
        self.slow_dist      = float(rospy.get_param("~safety_slow_distance_m", 0.70))
        self.charge_camera  = rospy.get_param("~charge_camera_name", "usb_cam_2")

        q_lr      = float(rospy.get_param("~q_lr",      0.05))
        q_gamma   = float(rospy.get_param("~q_gamma",   0.90))
        q_epsilon = float(rospy.get_param("~q_epsilon", 0.10))
        q_path    = rospy.get_param("~q_table_path",
                                    os.path.expanduser("~/.ros/duckierace_q_%s.json" % self.robot))

        self.agent = QAgent(lr=q_lr, gamma=q_gamma, epsilon=q_epsilon,
                            table_path=q_path)
        rospy.loginfo("[QDriver/%s] Q-table loaded from %s", self.robot, q_path)
        rospy.loginfo("\n%s", self.agent.policy_summary())

        # ── robot state ────────────────────────────────────────────────────
        self.battery      = 75.0
        self.selected_spd = self.start_speed
        self.lap_count    = 0.0
        self.in_fuel      = False
        self.in_gate      = False   # aggregated
        self.in_gate_cam  = False   # cam2-specific (used for gate detection)
        self.in_merge     = False
        self.local_brake  = True
        self.front_range  = 9.9
        self.lead_dist    = 9.9
        self.pose_valid   = False
        self.pose_xy      = (0.0, 0.0)
        self.last_update  = 0.0

        self.peers = {r: {"battery": 75.0, "in_fuel": False,
                           "pose": (0.0, 0.0), "pose_valid": False}
                      for r in self.all_robots if r != self.robot}

        # ── charging state machine ─────────────────────────────────────────
        self.charge_state   = ChargeState.IDLE
        self.seeking_fuel   = False
        self.leaving_charge = False
        self.waiting_fuel   = False
        self._leave_t       = 0.0
        self._brake_target  = None
        self._brake_sent    = False
        self._brake_ticks   = 0
        self._brake_retries = 0

        # ── race state ─────────────────────────────────────────────────────
        self.global_brake   = True
        self.brake_released = False
        self.start_time     = None

        # ── reward tracking ────────────────────────────────────────────────
        self._prev_lap    = 0.0
        self._prev_bat    = self.battery
        self._laps_since_save = 0
        self._collision_this_step = False

        # ── publishers ─────────────────────────────────────────────────────
        self.joy_pub  = rospy.Publisher("/%s/joy"           % self.robot, Joy,    queue_size=1)
        self.mode_pub = rospy.Publisher("/%s/driving_mode"  % self.robot, String, queue_size=1, latch=True)
        self.dec_pub  = rospy.Publisher("/%s/q_decision"    % self.robot, String, queue_size=1)

        # ── subscribers ────────────────────────────────────────────────────
        R = self.robot
        rospy.Subscriber("/%s/power_level"        % R, Float32, lambda m: self._set("battery",     float(m.data)))
        rospy.Subscriber("/%s/speed_percent"      % R, Float32, lambda m: self._set("selected_spd",float(m.data)))
        rospy.Subscriber("/%s/lap_count"          % R, Float32, lambda m: self._on_lap(float(m.data)))
        rospy.Subscriber("/%s/in_fuel_zone"       % R, Bool,    lambda m: self._set("in_fuel",     bool(m.data)))
        rospy.Subscriber("/%s/in_charge_gate_zone"% R, Bool,    lambda m: self._set("in_gate",     bool(m.data)))
        rospy.Subscriber("/%s/%s/in_charge_gate_zone" % (R, self.charge_camera),
                                                       Bool,    lambda m: self._set("in_gate_cam", bool(m.data)))
        rospy.Subscriber("/%s/in_merge_zone"      % R, Bool,    lambda m: self._set("in_merge",    bool(m.data)))
        rospy.Subscriber("/%s/local_brake"        % R, Bool,    lambda m: self._set("local_brake", bool(m.data)))
        rospy.Subscriber("/%s/front_range"        % R, Range,   lambda m: self._set("front_range", float(m.range)))
        rospy.Subscriber("/%s/perception/lead_car_distance" % R,
                                                       Float32, lambda m: self._set("lead_dist",   float(m.data)))
        rospy.Subscriber("/%s/tracked_pose"       % R, Pose2D,  self._cb_pose)
        rospy.Subscriber("/global_brake",              Bool,    self._cb_global_brake)

        for peer in list(self.peers.keys()):
            rospy.Subscriber("/%s/in_fuel_zone"  % peer, Bool,
                             self._make_peer_cb(peer, "in_fuel"))
            rospy.Subscriber("/%s/tracked_pose"  % peer, Pose2D,
                             self._make_peer_pose_cb(peer))

        rospy.Timer(rospy.Duration(1.0 / self.rate_hz), self._loop)
        self.mode_pub.publish(String(data="q_table"))
        rospy.loginfo("[QDriver/%s] started", self.robot)

    # ── helpers ───────────────────────────────────────────────────────────────

    def _set(self, attr, val):
        setattr(self, attr, val)
        self.last_update = rospy.Time.now().to_sec()

    def _on_lap(self, val):
        if val > self._prev_lap:
            gained = val - self._prev_lap
            reward = 10.0 * gained
            rospy.loginfo("[QDriver/%s] lap +%.0f  reward=%.1f", self.robot, gained, reward)
            self._update_q(reward)
            self._prev_lap = val
            self._laps_since_save += 1
            if self._laps_since_save >= SAVE_EVERY_N_LAPS:
                self.agent.save()
                self._laps_since_save = 0
        self.lap_count = val

    def _make_peer_cb(self, peer, field):
        def cb(msg):
            self.peers[peer][field] = bool(msg.data)
        return cb

    def _make_peer_pose_cb(self, peer):
        def cb(msg):
            self.peers[peer]["pose"]       = (float(msg.x), float(msg.y))
            self.peers[peer]["pose_valid"] = True
        return cb

    def _cb_pose(self, msg):
        self.pose_xy    = (float(msg.x), float(msg.y))
        self.pose_valid = True
        self.last_update = rospy.Time.now().to_sec()

    def _cb_global_brake(self, msg):
        if msg.data:
            self.brake_released = False
            self.start_time     = None
            self._reset_charge()
        elif self.global_brake:
            self.start_time = rospy.Time.now().to_sec()
        self.global_brake = bool(msg.data)

    # ── joy helpers ───────────────────────────────────────────────────────────

    @staticmethod
    def _joy(buttons=None):
        msg          = Joy()
        msg.header.stamp = rospy.Time.now()
        msg.buttons  = buttons if buttons is not None else [0] * 12
        msg.axes     = [0.0] * 8
        return msg

    def _press(self, buttons, *idxs):
        for i in idxs:
            if 0 <= i < len(buttons):
                buttons[i] = 1

    def _step_speed(self, target, buttons):
        cur = int(round((self.selected_spd - self.start_speed) / self.speed_step))
        tgt = int(round((float(np.clip(target, self.min_speed, self.max_speed))
                         - self.start_speed) / self.speed_step))
        if cur < tgt:
            self._press(buttons, BTN_L1)
        elif cur > tgt:
            self._press(buttons, BTN_R1)

    # ── brake retry ───────────────────────────────────────────────────────────

    def _clear_brake(self):
        self._brake_target = None
        self._brake_sent   = False
        self._brake_ticks  = 0
        self._brake_retries = 0

    def _brake_step(self, want: bool):
        """Returns (Joy, done). Retries with cap to avoid toggling."""
        if bool(self.local_brake) == want:
            self._clear_brake()
            return self._joy(), True

        if self._brake_target != want:
            self._brake_target  = want
            self._brake_sent    = False
            self._brake_ticks   = 0
            self._brake_retries = 0

        if self._brake_retries >= MAX_BRAKE_RETRIES:
            rospy.logwarn("[QDriver/%s] brake %s timeout — forcing",
                          self.robot, "on" if want else "off")
            self._clear_brake()
            return self._joy(), True

        if not self._brake_sent:
            self._brake_sent  = True
            self._brake_ticks = 0
            buttons = [0] * 12
            if want:
                self._press(buttons, BTN_X)
            elif self.in_fuel:
                self._press(buttons, BTN_X, BTN_B)
            else:
                self._press(buttons, BTN_X)
            return self._joy(buttons), False

        self._brake_ticks += 1
        if self._brake_ticks < BRAKE_CONFIRM_TICKS:
            return self._joy(), False

        self._brake_sent    = False
        self._brake_ticks   = 0
        self._brake_retries += 1
        rospy.logwarn("[QDriver/%s] brake retry %d (want=%s)",
                      self.robot, self._brake_retries, want)
        return self._joy(), False

    # ── charge state machine ──────────────────────────────────────────────────

    def _reset_charge(self):
        self.charge_state   = ChargeState.IDLE
        self.seeking_fuel   = False
        self.leaving_charge = False
        self.waiting_fuel   = False
        self._leave_t       = 0.0
        self._clear_brake()

    def _charge_control(self):
        if self.charge_state == ChargeState.LOCKING:
            joy, done = self._brake_step(True)
            if done:
                self.charge_state = ChargeState.CHARGING
                rospy.loginfo("[QDriver/%s] brake locked — charging", self.robot)
                b = [0] * 12
                self._press(b, BTN_Y)
                return self._joy(b), "charge_locking_done"
            return joy, "charge_locking"

        if self.charge_state == ChargeState.CHARGING:
            if self.battery >= self.charge_target:
                joy, done = self._brake_step(False)
                if done:
                    bat_gained = self.battery - self._prev_bat
                    self._update_q(float(bat_gained))   # reward = battery recovered
                    self._prev_bat = self.battery
                    self.charge_state   = ChargeState.IDLE
                    self.leaving_charge = True
                    self._leave_t       = rospy.Time.now().to_sec()
                    rospy.loginfo("[QDriver/%s] charged — leaving", self.robot)
                    b = [0] * 12
                    self._press(b, BTN_B)
                    return self._joy(b), "charge_done"
                return joy, "charge_done_releasing"
            b = [0] * 12
            self._press(b, BTN_Y)
            return self._joy(b), "charging"

        return self._joy(), "charge_idle"

    # ── Q-table helpers ───────────────────────────────────────────────────────

    def _current_zone(self) -> int:
        if self.in_fuel:   return ZONE_FUEL
        if self.in_gate_cam: return ZONE_GATE
        if self.in_merge:  return ZONE_MERGE
        return ZONE_TRACK

    def _wants_charge(self) -> bool:
        """Ask the Q-table whether to seek charging given current state."""
        zone   = self._current_zone()
        action = self.agent.select_action(self.battery, zone)
        return action == ACTION_CHARGE

    def _update_q(self, reward: float):
        zone = self._current_zone()
        self.agent.update(reward, self.battery, zone)

    # ── safety ────────────────────────────────────────────────────────────────

    def _nearest_peer_gap(self):
        if not self.pose_valid:
            return float("inf")
        ox, oy = self.pose_xy
        best = float("inf")
        for p in self.peers.values():
            if not p.get("pose_valid"):
                continue
            dx = p["pose"][0] - ox
            dy = p["pose"][1] - oy
            best = min(best, math.sqrt(dx*dx + dy*dy))
        return best

    def _safety(self):
        gap  = self._nearest_peer_gap()
        near = min(self.front_range, self.lead_dist, gap)
        if near <= self.hard_stop_m or gap <= self.stop_dist:
            return "stop"
        if near <= self.slow_dist:
            return "slow"
        return "clear"

    # ── main decide ───────────────────────────────────────────────────────────

    def _decide(self):
        buttons = [0] * 12
        now     = rospy.Time.now().to_sec()

        # active charge state machine
        if self.charge_state != ChargeState.IDLE:
            return self._charge_control()

        # navigating from gate to fuel zone
        if self.seeking_fuel:
            if self.in_fuel:
                self.seeking_fuel = False
                self.charge_state = ChargeState.LOCKING
                return self._charge_control()
            if self.in_gate_cam and any(p["in_fuel"] for p in self.peers.values()):
                self.waiting_fuel = True
                if not self.local_brake:
                    return self._brake_step(True)[0], "wait_peer_in_fuel"
                return self._joy(), "waiting_at_gate"
            if self.waiting_fuel and self.local_brake:
                self.waiting_fuel = False
                return self._brake_step(False)[0], "release_after_wait"
            self.waiting_fuel = False
            self._press(buttons, BTN_B)
            self._step_speed(self.seek_speed, buttons)
            return self._joy(buttons), "seek_fuel"

        # entered fuel zone wanting charge
        if self.in_fuel and self.battery < self.charge_target:
            self.charge_state = ChargeState.LOCKING
            return self._charge_control()

        # leaving charger
        if self.leaving_charge:
            if self.in_merge or self.in_fuel or (now - self._leave_t) > 5.0:
                self.leaving_charge = False
            else:
                self._press(buttons, BTN_B)
                return self._joy(buttons), "leaving_charge"

        # Q-table gate decision
        if self.in_gate_cam and not self.in_fuel and self._wants_charge():
            if self.local_brake:
                return self._brake_step(False)[0], "release_at_gate"
            self.seeking_fuel = True
            self._press(buttons, BTN_B)
            self._step_speed(self.seek_speed, buttons)
            return self._joy(buttons), "enter_gate"

        # give Q-table a small urgency penalty when battery is critical
        if self.battery <= self.critical_power:
            self._update_q(-0.5)

        # normal drive
        if self.local_brake:
            return self._brake_step(False)[0], "release_brake"
        self._step_speed(self.cruise, buttons)
        return self._joy(buttons), "drive"

    # ── control loop ──────────────────────────────────────────────────────────

    def _loop(self, _):
        if self.global_brake:
            if not self.local_brake:
                b = [0] * 12
                self._press(b, BTN_X)
                self.joy_pub.publish(self._joy(b))
            return

        if not self.brake_released:
            joy, done = self._brake_step(False)
            self.joy_pub.publish(joy)
            if done:
                self.brake_released = True
                rospy.loginfo("[QDriver/%s] race brake released", self.robot)
            return

        safety = self._safety()
        if safety == "stop":
            b = [0] * 12
            if not self.local_brake:
                self._press(b, BTN_X)
            self.joy_pub.publish(self._joy(b))
            self._update_q(-5.0)   # collision penalty
            return

        if safety == "slow":
            b = [0] * 12
            self._step_speed(self.min_speed, b)
            self.joy_pub.publish(self._joy(b))
            return

        joy_msg, reason = self._decide()
        self.joy_pub.publish(joy_msg)

        # publish decision for monitoring
        self.dec_pub.publish(String(data=json.dumps({
            "robot":       self.robot,
            "reason":      reason,
            "battery":     round(self.battery, 1),
            "zone":        self._current_zone(),
            "charge_state": self.charge_state.value,
            "seeking":     self.seeking_fuel,
            "local_brake": self.local_brake,
            "buttons":     list(joy_msg.buttons),
        })))


def main():
    QDriver()
    rospy.spin()


if __name__ == "__main__":
    main()
