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

        self.pwm_index = 0
        self.pwm_values = [30]
        
        # self.trim = [0.12,0.1,0.08] # henry
        self.trim = [0]
    def encoder_cb(self, msg):
        self.current_ticks["left"] = msg.left_ticks
        self.current_ticks["right"] = msg.right_ticks

    def run(self):
        while not rospy.is_shutdown() and self.pwm_index < len(self.pwm_values):
            pwm = self.pwm_values[self.pwm_index]
            trim = self.trim[self.pwm_index] if self.pwm_index < len(self.trim) else 0
            rospy.loginfo(f"Running both wheels at {pwm}% PWM")
            ramp_steps = 4
            ramp_duration = 0.2
            for i in range(1, ramp_steps + 1):
                scale = i / ramp_steps
                cmd = WheelsCmd()
                cmd.throttle_left = pwm / 100 * scale * (1 - trim)
                cmd.throttle_right = pwm / 100 * scale * (1 + trim)
                self.cmd_pub.publish(cmd)
                rospy.sleep(ramp_duration / ramp_steps)

            rospy.loginfo("left wheel dc: %.2f, right wheel trim: %.2f",  pwm / 100 * scale * (1 - trim),  pwm / 100 * scale * (1 + trim))

            rospy.sleep(2.0)  # Stabilize

            self.tick_history.clear()
            self.time_history.clear()

            start_time = rospy.Time.now().to_sec()
            initial_ticks = self.current_ticks.copy()

            while rospy.Time.now().to_sec() - start_time < self.sample_window:
                self.tick_history.append(self.current_ticks.copy())
                self.time_history.append(rospy.Time.now().to_sec())
                self.rate.sleep()

            final_ticks = self.current_ticks
            delta_time = self.time_history[-1] - self.time_history[0]

            for wheel in ["left", "right"]:
                delta_ticks = final_ticks[wheel] - initial_ticks[wheel]
                if delta_time > 0:
                    omega = (2 * 3.1416 * delta_ticks) / (self.resolution * delta_time)
                else:
                    omega = 0.0
                self.results.append((wheel, pwm, omega))
                rospy.loginfo(f"{wheel} @ {pwm}% → omega = {omega:.3f} rad/s")

            # Save tick/time history for debugging and speed ramp-up analysis
            with open(f"pwm_{pwm}_ticks.csv", "w", newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["time", "ticks_left", "ticks_right"])
                for t, ticks in zip(self.time_history, self.tick_history):
                    writer.writerow([t, ticks["left"], ticks["right"]])

            cmd = WheelsCmd() 
            cmd.throttle_left = 0
            cmd.throttle_right = 0
            self.cmd_pub.publish(cmd)
            
            while True:
                user_input = input("Enter 'y' to continue, 'r' to repeat: ").lower()
                if user_input == 'y':
                    self.pwm_index += 1
                    break
                elif user_input == 'r':
                    break

        self.save_results()

    def save_results(self, filename="pwm_balance_map.csv"):
        with open(filename, "w", newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["wheel", "pwm", "omega"])
            writer.writerows(self.results)
        rospy.loginfo(f"Saved results to {filename}")

if __name__ == "__main__":
    mapper = PWMSpeedMapper()
    rospy.sleep(1.0)

    try:
        mapper.run()
    except rospy.ROSInterruptException:
        pass
