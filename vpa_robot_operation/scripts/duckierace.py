#!/usr/bin/env python3
"""Robot-side line following and virtual battery handling for DuckieRace."""

import time
from collections import deque

import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Joy, Range
from std_msgs.msg import Bool, Float32, Int32

from color_detector import ColorDetector
from controller.acc_controller import acc_controller
from vpa_robot_interface.msg import WheelsEncoder


class LineFollower:
    """Follow the requested lane color and expose the simulated battery state."""

    TOF_STATUS_VALID = 9

    def __init__(self):
        rospy.init_node("duckierace_line_follower_node")

        # Initialize every value that a callback can read before creating any
        # subscriber or timer.  Camera callbacks can arrive immediately during
        # startup, which previously exposed partially initialized state.
        self.robot_name = rospy.get_param("~robot_name", "henry")
        self.bridge = CvBridge()
        self.detector = ColorDetector()

        self.lost_detect = False
        self.lost_since = None
        self.search_delay = 0.2
        self.search_period = 0.7
        self.search_linear_speed = 0.1
        self.search_angular_speed = 0.25

        self.default_speed = 0.25
        self.speed = self.default_speed
        self.speed_step = 0.02
        self.max_speed = 0.35
        self.min_speed = 0.15

        self.kp = 4.0
        self.kd = float(rospy.get_param("~kd", 0.4))
        self.last_error = 0.0
        self.last_angular_speed = 0.0
        self.lost_linear_speed = 0.10
        self.lost_angular_decay = 0.5
        self.lost_angular_limit = 0.4
        self.last_time = rospy.Time.now()

        self.default_color = "red"
        self.alt_color = "yellow"
        self.target_color = self.default_color
        self.red_mode_active = False
        self.red_line_last_seen = rospy.Time.now()
        self.h_row_ratio = 0.6
        self.stop_in_fuel = False

        # Logitech Cordless RumblePad 2 / main-PC controller contract.
        self.button_x = 0
        self.button_b = 2
        self.button_y = 3
        self.button_l1 = 4
        self.button_r1 = 5
        self.last_joy_time = 0.0
        self.last_brake_sent = False

        self.z_min = 0.0
        self.z_max = 1.5
        self.tof_status = 0
        self.tof_range = self.z_max + 0.1
        self.tof_history = deque(maxlen=5)
        self.tof_warn = False
        self.acc_desired_gap = float(
            rospy.get_param("~acc_desired_gap", 0.35))
        self.acc_dead_band = float(
            rospy.get_param("~acc_dead_band", 0.10))
        self.acc_slow_zone = float(
            rospy.get_param("~acc_slow_zone", 0.20))
        self.acc_hard_stop = float(
            rospy.get_param("~acc_hard_stop", 0.12))
        self.acc_min_factor = float(
            rospy.get_param("~acc_min_factor", 0.0))

        self.default_power_level = float(
            rospy.get_param("~initial_power", 75.0))
        self.power_level = self.default_power_level
        self.power_depletion_rate = 3.0
        self.low_power_threshold = float(
            rospy.get_param("~low_power_threshold", 20.0))
        self.in_fuel_zone = False
        self.charging = False
        self.last_charge_time = time.time()
        self.recharge_rate = 5.0
        self.real_spd = 0.0

        # The main-PC virtual driver owns charging by default.  The retained
        # robot-local policy is an explicit fallback for standalone operation.
        self.autonomous_mode = bool(
            rospy.get_param("~autonomous_mode", False))
        self.charge_target_soc = float(
            rospy.get_param("~charge_target_soc", 100.0))
        self.charge_safety_margin = float(
            rospy.get_param("~charge_safety_margin", 10.0))
        self.race_duration = float(
            rospy.get_param("~race_duration", 300.0))
        self.race_start_time = time.time()
        self.want_charge = False
        self.brake_retry_timer = None

        # Publishers must exist before callbacks can attempt to use them.
        self.cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.brake_pub = rospy.Publisher(
            "/{}/local_brake".format(self.robot_name), Bool, queue_size=1)
        self.power_pub = rospy.Publisher(
            "/{}/power_level".format(self.robot_name), Float32, queue_size=1)
        self.spd_pub = rospy.Publisher(
            "speed_percent", Float32, queue_size=1)

        # Subscribers and timers are deliberately created last.  Keep this
        # block after all state and publishers when adding future callbacks.
        self.tof_status_sub = rospy.Subscriber(
            "front_range_status", Int32, self.status_tof_callback,
            queue_size=1)
        self.tof_sub = rospy.Subscriber(
            "front_range", Range, self.tof_callback, queue_size=1)
        self.joy_sub = rospy.Subscriber(
            "joy", Joy, self.joy_callback, queue_size=1)
        self.image_sub = rospy.Subscriber(
            "robot_cam/image_raw", Image, self.image_callback, queue_size=1)
        self.sub_wheel_enc = rospy.Subscriber(
            "wheel_omega", WheelsEncoder, self.wheel_omega_cb, queue_size=1)
        self.fuel_sub = rospy.Subscriber(
            "in_fuel_zone", Bool, self.fuel_callback, queue_size=1)
        self.reset_power_speed_sub = rospy.Subscriber(
            "reset_power_speed", Bool, self.reset_power_speed_callback,
            queue_size=1)

        if self.autonomous_mode:
            self.brake_retry_timer = rospy.Timer(
                rospy.Duration(0.5), self.brake_retry_callback)
        self.fuel_timer = rospy.Timer(
            rospy.Duration(1.0), self.fuel_timer_callback)

        self.power_pub.publish(Float32(data=self.power_level))
        control_owner = "robot" if self.autonomous_mode else "main-PC driver"
        rospy.loginfo(
            "Line follower node initialized; charging authority: %s",
            control_owner)

    @staticmethod
    def _button_pressed(msg, index):
        """Safely read a Joy button from devices with different array sizes."""
        return 0 <= index < len(msg.buttons) and bool(msg.buttons[index])

    def status_tof_callback(self, msg):
        self.tof_status = msg.data

    def tof_callback(self, msg):
        if self.tof_status == self.TOF_STATUS_VALID:
            raw = np.clip(msg.range - 0.04, self.z_min, self.z_max)
        else:
            raw = self.z_max + 0.1
        self.tof_history.append(raw)
        self.tof_range = float(np.median(self.tof_history))

    def fuel_timer_callback(self, _event):
        if self.autonomous_mode:
            self.update_charge_decision()
            self.red_mode_active = self.want_charge

        if self.charging:
            rospy.loginfo_throttle(
                5.0, "[%s] Charging in fuel zone", self.robot_name)
            self.power_level = min(
                100.0, self.power_level + self.recharge_rate)
        self.power_pub.publish(Float32(data=self.power_level))

    def reset_power_speed_callback(self, msg):
        if not msg.data:
            return
        self.speed = self.default_speed
        self.power_level = self.default_power_level
        self.charging = False
        self.want_charge = False
        self.red_mode_active = False
        self.target_color = self.default_color
        self.race_start_time = time.time()
        self.last_time = rospy.Time.now()
        self.spd_pub.publish(Float32(data=self.speed))
        self.power_pub.publish(Float32(data=self.power_level))
        rospy.loginfo("Reset power and speed for %s", self.robot_name)

    def wheel_omega_cb(self, msg):
        self.real_spd = (msg.omega_left + msg.omega_right) / 2.0

    def fuel_callback(self, msg):
        self.in_fuel_zone = msg.data
        rospy.loginfo("[%s] Fuel zone status: %s",
                      self.robot_name, self.in_fuel_zone)

    def update_charge_decision(self):
        """Optional standalone predictive, time-aware charging policy."""
        remaining = self.race_duration - (
            time.time() - self.race_start_time)

        if self.charging:
            if self.power_level >= self.charge_target_soc:
                rospy.loginfo(
                    "[%s] Charge target reached, resuming lap",
                    self.robot_name)
                self.charging = False
                self.want_charge = False
            return

        depletion_rate = self.power_depletion_rate * self.speed
        if depletion_rate > 0:
            time_until_low = (
                self.power_level - self.low_power_threshold
            ) / depletion_rate
        else:
            time_until_low = float("inf")
        will_ever_need_charge = time_until_low < remaining

        needs_charge_now = self.power_level < (
            self.low_power_threshold + self.charge_safety_margin)
        charge_time_needed = (
            self.charge_target_soc - self.power_level
        ) / self.recharge_rate
        worth_it = remaining > charge_time_needed

        self.want_charge = (
            will_ever_need_charge and needs_charge_now and worth_it)

        if self.want_charge and self.in_fuel_zone:
            self.charging = True
            rospy.loginfo("[%s] Starting charge at %.1f%%",
                          self.robot_name, self.power_level)

    def brake_retry_callback(self, _event):
        if self.charging:
            return
        self.brake_pub.publish(Bool(data=False))

    def joy_callback(self, msg):
        now = time.time()
        if now - self.last_joy_time < 0.1:
            return
        self.last_joy_time = now

        if self._button_pressed(msg, self.button_l1):
            self.speed = min(self.speed + self.speed_step, self.max_speed)
            rospy.loginfo("Speed increased to: %.2f", self.speed)

        if self._button_pressed(msg, self.button_r1):
            self.speed = max(self.speed - self.speed_step, self.min_speed)
            rospy.loginfo("Speed decreased to: %.2f", self.speed)

        self.spd_pub.publish(Float32(data=self.speed))

        x_pressed = self._button_pressed(msg, self.button_x)
        b_pressed = self._button_pressed(msg, self.button_b)
        y_pressed = self._button_pressed(msg, self.button_y)

        if x_pressed and not self.last_brake_sent:
            if not self.in_fuel_zone:
                rospy.loginfo(
                    "Sending manual brake unlock to /%s/local_brake",
                    self.robot_name)
                self.brake_pub.publish(Bool(data=False))
                self.last_brake_sent = True
            elif b_pressed:
                rospy.loginfo(
                    "Sending manual brake unlock to /%s/local_brake "
                    "in fuel zone", self.robot_name)
                self.brake_pub.publish(Bool(data=False))
                self.last_brake_sent = True
        elif x_pressed and self.last_brake_sent:
            rospy.loginfo(
                "Sending manual brake lock to /%s/local_brake",
                self.robot_name)
            self.brake_pub.publish(Bool(data=True))
            self.last_brake_sent = False

        if not self.autonomous_mode:
            self.charging = bool(y_pressed and self.in_fuel_zone)
            self.red_mode_active = b_pressed

    def image_callback(self, msg):
        if self.stop_in_fuel:
            self.publish_twist(0.0, 0.0)
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as error:
            rospy.logerr("Image conversion error: %s", error)
            return

        if self.red_mode_active:
            alt_mask = self.detector.get_mask(bgr, self.alt_color)
            if np.sum(alt_mask) > 2000:
                self.target_color = self.alt_color
                self.red_line_last_seen = rospy.Time.now()
            else:
                time_since_alt = (
                    rospy.Time.now() - self.red_line_last_seen).to_sec()
                if time_since_alt > 1.0:
                    self.target_color = self.default_color
        else:
            self.target_color = self.default_color

        mask = self.detector.get_mask(bgr, self.target_color)
        height, width = mask.shape
        row = int(height * self.h_row_ratio)
        indices = np.where(mask[row, :] > 0)[0]

        if len(indices) == 0:
            if not self.lost_detect:
                rospy.logwarn("No line detected!")
            self.lost_detect = True
            # A Twist remains active at the wheel driver until replaced.  Stop
            # on every lost-line frame so the robot cannot continue blindly.
            self.publish_twist(0.0, 0.0)
            return

        self.lost_detect = False
        self.lost_since = None
        center_x = width // 2
        avg_x = np.mean(indices)
        error = (avg_x - center_x) / center_x

        now = rospy.Time.now()
        dt = (now - self.last_time).to_sec()
        derror = (error - self.last_error) / dt if dt > 0 else 0.0

        angular_speed = -self.kp * error - self.kd * derror
        angular_speed = np.clip(angular_speed, -1.2, 1.2)

        self.last_error = error
        self.last_angular_speed = angular_speed
        self.last_time = now

        if self.power_level < self.low_power_threshold:
            self.speed = self.min_speed
            self.spd_pub.publish(Float32(data=self.speed))

        acc_scale = acc_controller(
            self.acc_desired_gap,
            self.tof_range,
            dead_band=self.acc_dead_band,
            slow_zone=self.acc_slow_zone,
            z_stop=self.acc_hard_stop,
            min_factor=self.acc_min_factor,
        )
        self.publish_twist(self.speed * acc_scale, angular_speed)

        if self.real_spd > 0.1:
            depletion = self.power_depletion_rate * self.speed * dt
            self.power_level = max(0.0, self.power_level - depletion)

    def publish_twist(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    @staticmethod
    def run():
        rospy.spin()


if __name__ == "__main__":
    LineFollower().run()
