#! /usr/bin/env python3

import rospy

from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

class Joy_Control:

    def __init__(self):
        rospy.init_node('joy_global_brake')
        self.global_brake_pub = rospy.Publisher('/global_brake', Bool, queue_size=1)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        
    def joy_callback(self, msg):
        # Assuming button index 0 is the button to trigger global brake
        # this is usually the "A" button on an Xbox controller
        if msg.buttons[0] == 1:
            self.global_brake_pub.publish(Bool(data=True))
            rospy.loginfo("Global brake activated")
        elif msg.buttons[1] == 1:  # Assuming button index 1 is to release the brake   
            self.global_brake_pub.publish(Bool(data=False))
            rospy.loginfo("Global brake released")

if __name__ == '__main__':
    try:
        joy_control = Joy_Control()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

