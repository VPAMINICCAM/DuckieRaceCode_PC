#! /usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray
import os

class TrafficSignalCycleNode:

    def __init__(self):
        rospy.init_node('traffic_signal_cycle_node')

        self.cycle_time_sec = rospy.get_param('~cycle_time_sec', 100)

        self.signal_pub = rospy.Publisher('/green_phases', Int32MultiArray, queue_size=1)

        current_dir = os.path.dirname(os.path.abspath(__file__))
        pkg_dir = os.path.dirname(current_dir)
        PLANNING_FILE_PATH = os.path.join(pkg_dir, 'signal_planning/signal_plan_cycle.csv')

        self.green_phases = self.load_signal_planning(PLANNING_FILE_PATH)

        # we set a timer to publish at 1 second intervals
        self.elapsed_time = 0
        rospy.Timer(rospy.Duration(self.cycle_time_sec/100), self.timer_callback)
        rospy.loginfo("Traffic Signal Cycle Node started.")

    def timer_callback(self, event):
        self.elapsed_time += 1
        if self.elapsed_time >= 100:
            self.elapsed_time = 0
        green_phases = self.green_phases.get(self.elapsed_time, [])
        # append a 0 in the data that is used for other purposes
        # green_phases.append(0)
        if 0 not in green_phases:
            green_phases.append(0)
        self.signal_pub.publish(Int32MultiArray(data=green_phases))

    def load_signal_planning(self, file_path):
        with open(file_path, 'r') as file:
            lines = file.readlines()[1:]  # Skip the header line
            # the first column is time step, from 0 to 99, meaning in this porportion of cycle time which phases are green
            # so we build a dictionary mapping time step to list of green phases
            green_phases = {}
            for line in lines:
                parts = line.strip().split(',')
                time_step = int(parts[0])
                phases = [int(phase) for phase in parts[1:] if phase]
                green_phases[time_step] = phases
            # print the head few lines for debugging
            for k in list(green_phases.keys())[:5]:
                rospy.loginfo(f"Time step {k}: green phases {green_phases[k]}")
        return green_phases

if __name__ == '__main__':
    node = TrafficSignalCycleNode()
    rospy.spin()