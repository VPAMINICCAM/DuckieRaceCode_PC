#! /usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray
import os

class TrafficSignalNode:

    def __init__(self):
        rospy.init_node('traffic_signal_node')
        self.signal_pub = rospy.Publisher('/green_phases', Int32MultiArray, queue_size=1)

        current_dir = os.path.dirname(os.path.abspath(__file__))
        pkg_dir = os.path.dirname(current_dir)
        PLANNING_FILE_PATH = os.path.join(pkg_dir, 'signal_planning/signal_plan.csv')

        self.green_phases = self.load_signal_planning(PLANNING_FILE_PATH)
        
        self.always_green_phase = []

        self.phase_order = []
        self.current_order = 0
        self.switch_times = []
        self.elapsed_time = 0

        for phase in self.green_phases:
            phase_num = phase[0]
            green_time = phase[1]
            if green_time == -1:
                self.always_green_phase.append(phase_num)   
            else:
                self.phase_order.append(phase_num)
                self.switch_times.append(green_time)
        
        rospy.Timer(rospy.Duration(1), self.timer_callback)
        rospy.loginfo("Traffic Signal Node started.")

    def timer_callback(self, event):
        msg = Int32MultiArray()
        if type(self.phase_order[self.current_order]) == int:
            msg.data = self.always_green_phase + [self.phase_order[self.current_order]]
        else:
            msg.data = self.always_green_phase + self.phase_order[self.current_order]
        self.signal_pub.publish(msg)
        self.elapsed_time += 1
        if self.elapsed_time >= self.switch_times[self.current_order]:
            self.elapsed_time = 0
            self.current_order = (self.current_order + 1) % len(self.phase_order)
            rospy.loginfo(f"Switched to phase {self.phase_order[self.current_order]}")

    def load_signal_planning(self, file_path):
        with open(file_path, 'r') as file:
            lines = file.readlines()[1:]  # Skip the header line
            green_phases = [
                (int(line.split(',')[0]), int(line.split(',')[1]))
                for line in lines
            ]
        return green_phases

if __name__ == '__main__':
    node = TrafficSignalNode()
    rospy.spin()
