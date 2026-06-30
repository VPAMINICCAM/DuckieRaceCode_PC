#!/usr/bin/env python3
import rospy
import csv
from vpa_robot_interface.msg import WheelsEncoder, WheelsCmd
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
import os
class LoggerNode:
    def __init__(self):
        rospy.init_node("dead_reckon_logger")

        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.file_prefix = os.path.join(script_dir, rospy.get_param("~file_prefix", "IMU_ENCODER_LOG"))

        self.encoder_data = None
        self.imu_data = None
        self.throttle_left = 0
        self.throttle_right = 0
        self.omega_left = 0
        self.omega_right = 0

        rospy.Subscriber("/throttle",WheelsCmd,self.throttle_cb)
        rospy.Subscriber("/wheel_omega", WheelsEncoder, self.encoder_cb)
        rospy.Subscriber("/imu", Imu, self.imu_cb)
        rospy.Subscriber("/stop_log", Bool, self.stop_cb)
        rospy.Subscriber("/start_log", Bool, self.start_cb)

        self.data_log = []
        self.logging = False

        # rospy.loginfo("LoggerNode ready. Logging started.")
        self.pub_ready = rospy.Publisher("/logger_ready", Bool, queue_size=1)
        rospy.loginfo("LoggerNode ready. Ack...")
        for _ in range(5):
            rospy.sleep(0.5)
            self.pub_ready.publish(Bool(data=True))

    def encoder_cb(self, msg):
        self.encoder_data = msg

    def start_cb(self, msg):
        if msg.data:
            rospy.loginfo("Start signal received. Starting logging...")
            self.logging = True

    def throttle_cb(self,msg:WheelsCmd):
        self.throttle_left = msg.throttle_left
        self.throttle_right = msg.throttle_right

    def imu_cb(self, msg):
        self.imu_data = msg

    def stop_cb(self, msg):
        if msg.data:
            rospy.loginfo("Stop signal received. Saving log...")
            self.logging = False
            self.save_log()

    def save_log(self):
        filename = f"{self.file_prefix}.csv"
        with open(filename, "w", newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                "time","throttle_left","throttle_right",
                "omega_left", "omega_right",
                "ticks_left",
                "ticks_right",
                "gyro_x", "gyro_y", "gyro_z",
                "accel_x", "accel_y", "accel_z",
                "ori_x", "ori_y", "ori_z", "ori_w"
            ])
            writer.writerows(self.data_log)
        rospy.loginfo(f"Log saved to {filename}")

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and self.logging:
            if self.encoder_data and self.imu_data:
                t = rospy.Time.now().to_sec()
                row = [
                    t,
                    self.throttle_left,
                    self.throttle_right,
                    self.encoder_data.omega_left,
                    self.encoder_data.omega_right,
                    self.encoder_data.left_ticks,
                    self.encoder_data.right_ticks,
                    self.imu_data.angular_velocity.x,
                    self.imu_data.angular_velocity.y,
                    self.imu_data.angular_velocity.z,
                    self.imu_data.linear_acceleration.x,
                    self.imu_data.linear_acceleration.y,
                    self.imu_data.linear_acceleration.z,
                    self.imu_data.orientation.x,
                    self.imu_data.orientation.y,
                    self.imu_data.orientation.z,
                    self.imu_data.orientation.w
                ]
                self.data_log.append(row)
            rate.sleep()

if __name__ == '__main__':
    node = LoggerNode()
    node.run()
