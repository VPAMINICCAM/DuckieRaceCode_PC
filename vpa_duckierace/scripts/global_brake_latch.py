#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool


def main():
    rospy.init_node("global_brake_latch")
    brake = rospy.get_param("~brake", False)
    pub = rospy.Publisher("/global_brake", Bool, queue_size=1, latch=True)
    rospy.sleep(0.5)
    pub.publish(Bool(data=brake))
    rospy.loginfo("Published latched /global_brake=%s", brake)
    rospy.spin()


if __name__ == "__main__":
    main()
