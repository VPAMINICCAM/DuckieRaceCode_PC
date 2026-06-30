#! /usr/bin/env python3
import rospy
import os
from vpa_robot_operation.srv import AssignTask, AssignTaskResponse
from std_msgs.msg import Bool
class Dispatcher:
    def __init__(self):
        rospy.init_node('task_assigner')
        self.task_dict = {}
        self.load_tasks()

        self.task_counter = 1
        self.service = rospy.Service('center_manager/assign_task', AssignTask, self.assign_task_callback)

        self.reset_sub = rospy.Subscriber('/reset_task',Bool,self.reset_cb)
        
    def reset_cb(self,msg):
        if msg.data:
            self.task_counter = 1
            rospy.loginfo('Task Counter Reset')
                
    def assign_task_callback(self, req): 
        rospy.loginfo(f"Received task assignment request from robot {req.robot_id}")
        resp = AssignTaskResponse()
        if self.task_counter in self.task_dict.keys():
            resp.entry_path = self.task_dict[self.task_counter][0]
            resp.loop_path = self.task_dict[self.task_counter][1]
            resp.loop_count = self.task_dict[self.task_counter][2]
            resp.exit_path = self.task_dict[self.task_counter][3]
            self.task_counter += 1
        else:
            resp.entry_path = []
            resp.loop_path = []
            resp.loop_count = 0
            resp.exit_path = []
            rospy.loginfo("No more tasks available.")
        return resp

    def load_tasks(self):
        current_dir         = os.path.dirname(os.path.abspath(__file__))
        pkg_dir             = os.path.dirname(current_dir)
        PLANNING_FILE_PATH  = os.path.join(pkg_dir, 'task/task.csv')
        cur_task_id = None
        with open(PLANNING_FILE_PATH, 'r') as file:
            lines = file.readlines()  # Skip the header line
            for line in lines:
                parts = line.strip().split(',')
                line_info = str(parts[0])
                if line_info  == 'task_id':
                    this_task = ([],[],0,[])
                    self.task_dict[int(parts[1])] = this_task
                    cur_task_id = int(parts[1])
                    print(f"Loaded task {cur_task_id}")
                elif line_info == 'entry_path':
                    entry_path = [int(i) for i in parts[1:]]
                    self.task_dict[cur_task_id][0].extend(entry_path)
                elif line_info == 'loop_path':
                    loop_path = [int(i) for i in parts[1:]]
                    self.task_dict[cur_task_id][1].extend(loop_path)
                elif line_info == 'loop_count':
                    loop_count = int(parts[1])
                    self.task_dict[cur_task_id] = (self.task_dict[cur_task_id][0], self.task_dict[cur_task_id][1], loop_count, self.task_dict[cur_task_id][3])
                elif line_info == 'exit_path':
                    exit_path = [int(i) for i in parts[1:]]
                    self.task_dict[cur_task_id][3].extend(exit_path)
        
        rospy.loginfo(f"Loaded tasks: {self.task_dict}")

if __name__ == "__main__":
    dispatcher = Dispatcher()
    rospy.loginfo("Task dispatcher is ready to assign tasks.")
    rospy.spin()