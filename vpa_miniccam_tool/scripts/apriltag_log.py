#!/usr/bin/env python3

import os
import shutil
import rospy
import cv2
import csv
import numpy as np
from dt_apriltags import Detector
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from vpa_robot_interface.msg import WheelsCmd
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool
class AprilTagLogger:
    def __init__(self):
        rospy.init_node('apriltag_logger')

        self.bridge = CvBridge()
        self.camera_matrix = np.array([
            [514.377730, 0.0, 650.987233],
            [0.0, 516.015468, 353.825446],
            [0.0, 0.0, 1.0]
        ])
        self.dist_coeffs = np.array([-0.004775, -0.024952, -0.003004, -0.000447, 0.0])
        self.tag_size = 0.12  # meters

        self.detector = Detector(families='tag36h11')
        self.logging = False
        self.image_counter = 0
        self.image_folder = os.path.join(os.path.dirname(__file__), "img")
        self.csv_file = os.path.join(os.path.dirname(__file__), "tag_detections.csv")

        if os.path.exists(self.image_folder):
            shutil.rmtree(self.image_folder)
        os.makedirs(self.image_folder)
        self.prev_moving = False
        rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        rospy.Subscriber("/throttle", WheelsCmd, self.throttle_callback)
        self.stop_pub = rospy.Publisher('/stop_log',Bool,queue_size=1)
        rospy.loginfo("AprilTagLogger ready.")

    def throttle_callback(self, msg):
        moving = msg.throttle_left > 0 or msg.throttle_right > 0
  
        if moving and not self.prev_moving:
            self.logging = True
            rospy.loginfo("Throttle started, logging frames...")
        elif not moving and self.logging:
            self.logging = False
            msg = Bool()
            msg.data = True
            self.stop_pub.publish(msg)
            rospy.loginfo("Throttle stopped, running detection...")
            self.run_detection()
    def image_callback(self, msg):
        if not self.logging:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logerr(f"CV Bridge error: {e}")
            return

        filename = os.path.join(self.image_folder, f"frame_{self.image_counter:06d}.jpg")
        cv2.imwrite(filename, frame)
        self.image_counter += 1

    def run_detection(self):
        with open(self.csv_file, "w", newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["time", "tx", "ty", "tz", "qx", "qy", "qz", "qw", "tag_id"])

            for i in range(self.image_counter):
                img_path = os.path.join(self.image_folder, f"frame_{i:06d}.jpg")
                frame = cv2.imread(img_path)
                if frame is None:
                    continue

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                t_now = rospy.Time.now().to_sec()

                detections = self.detector.detect(
                    gray,
                    estimate_tag_pose=True,
                    camera_params=(
                        self.camera_matrix[0, 0],
                        self.camera_matrix[1, 1],
                        self.camera_matrix[0, 2],
                        self.camera_matrix[1, 2]
                    ),
                    tag_size=self.tag_size
                )

                for det in detections:
                    pose_t = det.pose_t.flatten()
                    quat = R.from_dcm(det.pose_R).as_quat()  # x, y, z, w
                    writer.writerow([t_now, *pose_t, *quat, det.tag_id])

        rospy.loginfo(f"Detection finished. Saved to {self.csv_file}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = AprilTagLogger()
    node.run()
