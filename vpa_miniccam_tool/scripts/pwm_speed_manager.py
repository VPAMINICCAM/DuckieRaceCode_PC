#!/usr/bin/env python3
import rospy
import csv
import time
from vpa_robot_interface.msg import WheelsCmd, WheelsEncoder

class PWMSpeedMapper:
    def __init__(self):
        rospy.init_node("pwm_speed_mapper")

        self.cmd_pub = rospy.Publisher("/throttle", WheelsCmd, queue_size=1)
        rospy.Subscriber("/wheel_omega", WheelsEncoder, self.encoder_cb)

        self.current_ticks = {"left": 0, "right": 0}
        self.tick_history = []
        self.time_history = []
        self.results = []

        self.resolution = 135  # encoder ticks per rev (single phase)
        self.sample_window = 1.0  # seconds
        self.rate = rospy.Rate(10)  # 10 Hz tick recording

    def encoder_cb(self, msg):
        self.current_ticks["left"] = msg.left_ticks
        self.current_ticks["right"] = msg.right_ticks

    def run_sweep(self, wheel):
        assert wheel in ["left", "right"]
        other = "right" if wheel == "left" else "left"

        pwm_values = list(range(30, 101, 10))  # 30 to 100% throttle

        for pwm in pwm_values:
            rospy.loginfo(f"Testing {wheel} wheel at {pwm}% PWM")

            # Set command
            cmd = WheelsCmd()
            setattr(cmd, f"throttle_{wheel}", pwm / 100.0)
            setattr(cmd, f"throttle_{other}", 0.0)
            self.cmd_pub.publish(cmd)

            # Let it stabilize
            rospy.sleep(5.0)

            # Start logging
            self.tick_history.clear()
            self.time_history.clear()

            start_time = rospy.Time.now().to_sec()
            initial_ticks = self.current_ticks[wheel]

            while rospy.Time.now().to_sec() - start_time < self.sample_window:
                self.tick_history.append(self.current_ticks[wheel])
                self.time_history.append(rospy.Time.now().to_sec())
                self.rate.sleep()

            final_ticks = self.current_ticks[wheel]
            delta_ticks = final_ticks - initial_ticks
            delta_time = self.time_history[-1] - self.time_history[0]

            if delta_time > 0:
                omega = (2 * 3.1416 * delta_ticks) / (self.resolution * delta_time)
            else:
                omega = 0.0

            self.results.append((wheel, pwm, omega))
            rospy.loginfo(f"{wheel} @ {pwm}% → omega = {omega:.3f} rad/s")

            # Stop motors before next test
            stop_cmd = WheelsCmd()
            self.cmd_pub.publish(stop_cmd)
            rospy.sleep(1.0)

    def save_results(self, filename="pwm_speed_map.csv"):
        with open(filename, "w", newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["wheel", "pwm", "omega"])
            writer.writerows(self.results)
        rospy.loginfo(f"Saved results to {filename}")

if __name__ == "__main__":
    mapper = PWMSpeedMapper()
    rospy.sleep(1.0)  # Wait for encoder messages to start

    try:
        mapper.run_sweep("left")
        mapper.run_sweep("right")
        mapper.save_results()
    except rospy.ROSInterruptException:
        pass
