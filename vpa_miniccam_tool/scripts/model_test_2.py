#!/usr/bin/env python3
import rospy
from vpa_robot_interface.msg import WheelsEncoder, WheelsCmd
from std_msgs.msg import Bool
import math

class ModelTest:
    def __init__(self):
        rospy.init_node("model_test")

        self.cmd_pub = rospy.Publisher("/throttle", WheelsCmd, queue_size=1)
        self.encoder_sub = rospy.Subscriber("/wheel_omega", WheelsEncoder, self.encoder_cb)
        self.pub_stop = rospy.Publisher("/stop_log", Bool, queue_size=1)
        self.sub_logger_ready = rospy.Subscriber("/logger_ready", Bool, self.logger_ready_cb)

        self.pub_start = rospy.Publisher("/start_log", Bool, queue_size=1)

        self.pub_local_brake = rospy.Publisher("/local_brake", Bool, queue_size=1)
        self.pub_global_brake = rospy.Publisher("/global_brake", Bool, queue_size=1)

        rospy.loginfo("Waiting for logger ready message...")
        rospy.wait_for_message("/logger_ready", Bool)
        rospy.loginfo("Logger is ready. Starting model test.")

        self.logger_ready = False
        self.get_enc = False

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.left_ticks = 0
        self.right_ticks = 0

        # Predefined throttle test cases
        self.test_cases = [
            (0.3, 0.3),
            (0.5, 0.3),
            (0.5, 0.5)
        ]

        self.pub_local_brake.publish(False)
    
        self.pub_global_brake.publish(False)    
        rospy.loginfo("Local and global brakes released.")
        # Timer to update position
        self.timer = rospy.Timer(rospy.Duration(0.1), self.update_position)

    def logger_ready_cb(self, msg):
        self.logger_ready = msg.data

    def encoder_cb(self, msg):
        self.get_enc = True
        self.left_ticks = msg.left_ticks
        self.right_ticks = msg.right_ticks

    def reset_position(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        rospy.loginfo("Position reset.")

    def update_position(self, event):
        TICKS_PER_REV = 135
        RADIUS = 0.0318
        BASE = 0.1

        l_dist = (self.left_ticks * 2 * math.pi * RADIUS) / TICKS_PER_REV
        r_dist = (self.right_ticks * 2 * math.pi * RADIUS) / TICKS_PER_REV

        dist = (l_dist + r_dist) / 2
        dtheta = (r_dist - l_dist) / BASE

        self.x += dist * math.cos(self.theta)
        self.y += dist * math.sin(self.theta)
        self.theta += dtheta

    def run(self):
        rate = rospy.Rate(20)
        self.pub_start.publish(True)
        rospy.loginfo("Model test started in 3 seconds...")
        rospy.sleep(3)
        while not rospy.is_shutdown():
            if self.logger_ready and self.get_enc:
                for left, right in self.test_cases:
                    cmd = WheelsCmd()
                    cmd.throttle_left = left
                    cmd.throttle_right = right
                    self.cmd_pub.publish(cmd)
                    rospy.loginfo(f"Sent throttle: L={left}, R={right}")
                    rospy.sleep(5.0)

                    self.cmd_pub.publish(WheelsCmd())  # stop
                    rospy.loginfo("Robot stopped. Please reset to start position.")
                    input("Press ENTER to continue...")
                    self.reset_position()

                self.pub_stop.publish(True)
                rospy.loginfo("All tests done.")
                break
            else:
                rate.sleep()

if __name__ == "__main__":
    try:
        test = ModelTest()
        test.run()
    except rospy.ROSInterruptException:
        pass
