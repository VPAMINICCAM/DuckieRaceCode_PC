#!/usr/bin/env python3
import rospy
from vpa_robot_interface.msg import WheelsEncoder,WheelsCmd
import os
import math
from std_msgs.msg import Bool

class ModelTest:
    def __init__(self):
        rospy.init_node("model_test")

        self.cmd_pub = rospy.Publisher("/throttle", WheelsCmd, queue_size=1)
        self.encoder_sub = rospy.Subscriber("/wheel_omega", WheelsEncoder, self.encoder_cb)
        self.pub_stop = rospy.Publisher("/stop_log", Bool, queue_size=1)
        self.logger_ready = False
        self.sub_enc_imu_logger = rospy.Subscriber("/logger_ready", Bool, self.logger_ready_cb)
        # Wait for logger ready message
        rospy.loginfo("Waiting for logger ready message...")
        rospy.wait_for_message("/logger_ready", Bool)
        rospy.loginfo("Logger is ready. Starting model test.")
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # Add timer for position calculation (10 Hz)
        self.timer = rospy.Timer(rospy.Duration(0.1), self.calculate_position)

        self.left_ticks = 0
        self.right_ticks = 0

        self.get_enc = False

        self.left_throttle_range = [0.3,0.4,0.5,0.6,0.7]
        self.right_throttle_range = [0.3,0.4,0.5,0.6,0.7]
    
    def logger_ready_cb(self, msg):
        self.logger_ready = msg.data
        rospy.loginfo("Control Node: Logger ready status: %s", self.logger_ready)
    
    def encoder_cb(self, msg):
        self.get_enc = True
        self.left_ticks     = msg.left_ticks
        self.right_ticks    = msg.right_ticks

    def reset_position(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        rospy.loginfo("Position reset to (0, 0, 0)")

    def calculate_position(self, event):
        # Constants for the robot
        TICKS_PER_REVOLUTION = 135      # Adjust based on your encoder
        WHEEL_RADIUS = 0.0318           # in meters
        WHEEL_BASE = 0.1                # distance between wheels in meters
        
        # Calculate distance traveled by each wheel
        left_distance = (self.left_ticks * 2 * math.pi * WHEEL_RADIUS) / TICKS_PER_REVOLUTION
        right_distance = (self.right_ticks * 2 * math.pi * WHEEL_RADIUS) / TICKS_PER_REVOLUTION
        
        # Calculate robot's linear and angular movement
        distance = (left_distance + right_distance) / 2
        angle = (right_distance - left_distance) / WHEEL_BASE
        
        # Update position (x, y) and orientation (theta)
        self.x += distance * math.cos(self.theta)
        self.y += distance * math.sin(self.theta)
        self.theta += angle

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.logger_ready and self.get_enc:
                for left_throttle in self.left_throttle_range:
                    for right_throttle in self.right_throttle_range:
                        cmd = WheelsCmd()
                        cmd.throttle_left = left_throttle
                        cmd.throttle_right = right_throttle
                        self.cmd_pub.publish(cmd)
                        rospy.loginfo(f"Published command: {cmd}")
                        rospy.sleep(5.0)  # Sleep for 5 seconds

                        # need to return to a closer position of starting point and roughly same angle of start
                        cmd = WheelsCmd()
                        cmd.throttle_left = 0.0
                        cmd.throttle_right = 0.0
                        self.cmd_pub.publish(cmd)
                        rospy.loginfo("Stopping robot after test.")

                        rospy.loginfo("Please manually move the robot back to starting position.")
                        input("Press 'y' when ready to continue: ")


                        self.reset_position()
                
            
                self.pub_stop.publish(True)
                rospy.loginfo("Test completed. Stopping robot and resetting position.")
                break
            else:
                pass

if __name__ == "__main__":
    try:
        model_test = ModelTest()
        model_test.run()
    except rospy.ROSInterruptException:
        pass