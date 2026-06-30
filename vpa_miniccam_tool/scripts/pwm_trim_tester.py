#!/usr/bin/env python3
import rospy
import csv
import time
from vpa_robot_interface.msg import WheelsCmd, WheelsEncoder

# Measured trim ratio: right wheel needs higher PWM to match left
# Example: at PWM 80, left=15.9, right=10.2 => ratio = 1.56
PWM_TRIM_RATIO = 1.56  # right = left * ratio

class PWMTrimTester:
    def __init__(self):
        rospy.init_node("pwm_trim_tester")

        self.cmd_pub = rospy.Publisher("/throttle", WheelsCmd, queue_size=1)
        rospy.Subscriber("/wheel_omega", WheelsEncoder, self.encoder_cb)

        self.current_ticks = {"left": 0, "right": 0}
        self.tick_history = []
        self.time_history = []
        self.results = []

        self.resolution = 135
        self.sample_window = 1.0
        self.rate = rospy.Rate(10)
        self.pwm_index = 0
        self.pwm_values = list(range(40, 71, 10))  # test only in moving range

    def encoder_cb(self, msg):
        self.current_ticks["left"] = msg.left_ticks
        self.current_ticks["right"] = msg.right_ticks

    def run(self):
        while not rospy.is_shutdown() and self.pwm_index < len(self.pwm_values):
            base_pwm = self.pwm_values[self.pwm_index]
            trimmed_pwm_right = min(100, int(base_pwm * PWM_TRIM_RATIO))

            rospy.loginfo(f"Testing: left={base_pwm}%, right~={trimmed_pwm_right}%")

            cmd = WheelsCmd()
            cmd.throttle_left = base_pwm / 100.0
            cmd.throttle_right = trimmed_pwm_right / 100.0
            self.cmd_pub.publish(cmd)

            rospy.sleep(2.0)  # Let it stabilize
            self.tick_history.clear()
            self.time_history.clear()
            initial_ticks = self.current_ticks.copy()
            start_time = rospy.Time.now().to_sec()

            while rospy.Time.now().to_sec() - start_time < self.sample_window:
                self.tick_history.append(self.current_ticks.copy())
                self.time_history.append(rospy.Time.now().to_sec())
                self.rate.sleep()

            final_ticks = self.current_ticks
            delta_time = self.time_history[-1] - self.time_history[0]

            tick_diff = {
                wheel: final_ticks[wheel] - initial_ticks[wheel] for wheel in ["left", "right"]
            }
            self.results.append((base_pwm, tick_diff["left"], tick_diff["right"]))
            rospy.loginfo(f"Ticks @ {base_pwm}% base: L={tick_diff['left']}, R={tick_diff['right']}")

            with open(f"trimtest_{base_pwm}_ticks.csv", "w", newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["time", "ticks_left", "ticks_right"])
                for t, ticks in zip(self.time_history, self.tick_history):
                    writer.writerow([t, ticks["left"], ticks["right"]])
                        # Stop motors before next test
            stop_cmd = WheelsCmd()
            self.cmd_pub.publish(stop_cmd)
            while True:
                user_input = input("Enter 'y' to continue, 'r' to repeat: ").lower()
                if user_input == 'y':
                    self.pwm_index += 1
                    break
                elif user_input == 'r':
                    break

        self.save_results()

    def save_results(self, filename="trim_test_summary.csv"):
        with open(filename, "w", newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["base_pwm", "left_ticks", "right_ticks"])
            writer.writerows(self.results)
        rospy.loginfo(f"Saved summary to {filename}")

if __name__ == "__main__":
    tester = PWMTrimTester()
    rospy.sleep(1.0)
    try:
        tester.run()
    except rospy.ROSInterruptException:
        pass
