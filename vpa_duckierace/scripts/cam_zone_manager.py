#!/usr/bin/env python3

import rospy
import yaml
import cv2
import os
import time
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from cv_bridge import CvBridge
from dt_apriltags import Detector as ATDetector
import matplotlib.pyplot as plt

class CamZoneManager:
    def __init__(self, zone_config_path_default, robot_tag_config_path):
        rospy.init_node('cam_zone_manager', anonymous=True)
        self.bridge = CvBridge()
        self.got_image = False
        # Zone visualization is useful while calibrating, but it must never
        # stop the image callback that publishes fuel/gate state during a race.
        self.visualize_zones_on_start = rospy.get_param("~visualize_zones", False)

        # Load configs
        cam_config_path = rospy.get_param('~cam_config_path', None)
        with open(cam_config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        zone_config_path = rospy.get_param('~zone_config_path', zone_config_path_default)
        with open(zone_config_path, 'r') as f:
            self.zone_config = yaml.safe_load(f)

        robot_tag_config_path = rospy.get_param('~robot_tag_config_path', robot_tag_config_path)
        with open(robot_tag_config_path, 'r') as f:
            self.robot_tags = yaml.safe_load(f)

        self.image_topic = self.zone_config['image_topic']
        self.has_finish = self.zone_config.get('has_finish_line', False)
        self.finish_line = self.zone_config.get('finish_line', None)
        self.cam_name = self.zone_config.get('camera_name', 'cam')

        self.detector = ATDetector(families='tag36h11')
        self.tag_size = 0.12

        camera_matrix = self.config['camera_matrix']
        data = camera_matrix['data'] if isinstance(camera_matrix, dict) else camera_matrix[0]
        self.fx, self.fy, self.cx, self.cy = data[0], data[4], data[2], data[5]

        self.merge_zones = self.zone_config.get('merge_zones', [])
        self.fuel_zone = self.zone_config.get('fuel_zone', None)
        self.charge_gate_zone = self.zone_config.get('charge_gate_zone', None)  # NEW
        self.ignore_zones = self.zone_config.get('ignore_zones', [])  # NEW
        self.tag_ground_offset = self.zone_config.get('tag_ground_offset', {'dx': 0.0, 'dy': 0.0})

        self.fuel_publishers = {}
        self.merge_publishers = {}
        self.charge_gate_publishers = {}  # NEW
        self.tag_visible_publishers = {}
        self.lap_publishers = {}
        self.tag_to_robot = {v: k for k, v in self.robot_tags.items()}

        self.last_positions = {}
        self.lap_counts = {}

        for robot_name, tag_id in self.robot_tags.items():
            self.fuel_publishers[tag_id] = rospy.Publisher(f"/{robot_name}/{self.cam_name}/in_fuel_zone", Bool, queue_size=1)
            self.merge_publishers[tag_id] = rospy.Publisher(f"/{robot_name}/{self.cam_name}/in_merge_zone", Bool, queue_size=1)
            self.charge_gate_publishers[tag_id] = rospy.Publisher(f"/{robot_name}/{self.cam_name}/in_charge_gate_zone", Bool, queue_size=1)  # NEW
            self.tag_visible_publishers[tag_id] = rospy.Publisher(f"/{robot_name}/{self.cam_name}/tag_visible", Bool, queue_size=1)
            self.lap_publishers[tag_id] = rospy.Publisher(f"/{robot_name}/{self.cam_name}/lap_count", Float32, queue_size=1)
            self.last_positions[tag_id] = None
            self.lap_counts[tag_id] = 0.0

        self.merge_zone_status = [{'tag_ids': set(), 'entry_times': {}} for _ in self.merge_zones]
        self.tags_in_merge = set()
        self.tags_in_fuel = set()
        self.tags_in_charge_gate = set()  # NEW

        self.image_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        self.reset_laps_sub = rospy.Subscriber("/reset_laps", Bool, self.reset_laps_callback)
        rospy.loginfo(f"CamZoneManager for {self.cam_name} started")

    def reset_laps_callback(self, msg):
        if not msg.data:
            return
        for tag_id in self.lap_counts:
            self.lap_counts[tag_id] = 0.0
            self.last_positions[tag_id] = None
            self.lap_publishers[tag_id].publish(Float32(data=0.0))
        self.last_lap_time = {}
        rospy.loginfo(f"Reset lap counts for {self.cam_name}")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if not self.got_image:
            self.got_image = True
            self.first_image = cv_image
            rospy.loginfo(f"Received first image on {self.image_topic}")
            if self.visualize_zones_on_start:
                self.visualize_zones()

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray = self.apply_ignore_mask(gray)  # NEW
        detections = self.detector.detect(gray)
        now = rospy.Time.now().to_sec()
        current_in_fuel = set()
        visible_tags = {det.tag_id for det in detections if det.tag_id in self.tag_to_robot}
        for tag_id in self.tag_visible_publishers:
            self.tag_visible_publishers[tag_id].publish(Bool(data=(tag_id in visible_tags)))
        
        for zone_idx, zone in enumerate(self.merge_zones):
            current_status = self.merge_zone_status[zone_idx]
            inside_now = set()

            for det in detections:
                tag_id = det.tag_id
                if tag_id not in self.tag_to_robot:
                    continue
                center = self.tag_ground_point(det)

                if self.is_point_in_zone(center, zone):
                    inside_now.add(tag_id)
                    if tag_id not in current_status['entry_times']:
                        current_status['entry_times'][tag_id] = now
                        rospy.loginfo(f"Tag {tag_id} ENTERED MERGE zone {zone_idx}")
                elif tag_id in current_status['tag_ids']:
                    rospy.loginfo(f"Tag {tag_id} EXITED MERGE zone {zone_idx}")

            if inside_now:
                first_tag = min(inside_now, key=lambda tid: current_status['entry_times'].get(tid, now))
            else:
                first_tag = None

            exited_tags = current_status['tag_ids'] - inside_now
            for tag_id in exited_tags:
                current_status['entry_times'].pop(tag_id, None)

            current_status['tag_ids'] = inside_now

        current_in_merge = set()
        for status in self.merge_zone_status:
            current_in_merge.update(status['tag_ids'])
        for tag_id in self.merge_publishers:
            self.merge_publishers[tag_id].publish(Bool(data=(tag_id in current_in_merge)))
        self.tags_in_merge = current_in_merge

        # Fuel zone logic
        for det in detections:
            tag_id = det.tag_id
            if tag_id not in self.tag_to_robot:
                continue
            center = self.tag_ground_point(det)
            if self.is_point_in_zone(center, self.fuel_zone):
                current_in_fuel.add(tag_id)
                if tag_id not in self.tags_in_fuel:
                    rospy.loginfo(f"Tag {tag_id} ENTERED FUEL zone")
            else:
                if tag_id in self.tags_in_fuel:
                    rospy.loginfo(f"Tag {tag_id} EXITED FUEL zone")
        for tag_id in self.fuel_publishers:
            self.fuel_publishers[tag_id].publish(Bool(data=(tag_id in current_in_fuel)))
        self.tags_in_fuel = current_in_fuel

        # Charge gate zone logic (NEW)
        current_in_gate = set()
        if self.charge_gate_zone:
            for det in detections:
                tag_id = det.tag_id
                if tag_id not in self.tag_to_robot:
                    continue
                center = self.tag_ground_point(det)
                if self.is_point_in_zone(center, self.charge_gate_zone):
                    current_in_gate.add(tag_id)
                    if tag_id not in self.tags_in_charge_gate:
                        rospy.loginfo(f"Tag {tag_id} ENTERED CHARGE GATE")
                else:
                    if tag_id in self.tags_in_charge_gate:
                        rospy.loginfo(f"Tag {tag_id} EXITED CHARGE GATE")
        for tag_id in self.charge_gate_publishers:
            self.charge_gate_publishers[tag_id].publish(Bool(data=(tag_id in current_in_gate)))

        self.tags_in_charge_gate = current_in_gate

        # Lap counting logic
        if self.has_finish and self.finish_line:
            pos_key = 'x' if self.finish_line['direction'] in ['left_to_right', 'right_to_left'] else 'y'
            axis_idx = 0 if pos_key == 'x' else 1
            threshold = self.finish_line['x']

            for det in detections:

                tag_id = det.tag_id
                pos = self.tag_ground_point(det)[axis_idx]
                last_pos = self.last_positions.get(tag_id)
                last_lap_time = getattr(self, 'last_lap_time', {})
                now_time = rospy.Time.now().to_sec()
                dead_time = 2.0  # seconds

                if last_pos is not None:
                    crossed = (
                        (last_pos < threshold and pos >= threshold) if self.finish_line['direction'] in ['left_to_right', 'top_to_bottom']
                        else (last_pos > threshold and pos <= threshold)
                    )
                    last_time = last_lap_time.get(tag_id, 0)
                    if crossed and (now_time - last_time > dead_time):
                        self.lap_counts[tag_id] += 0.5
                        robot = self.tag_to_robot.get(tag_id)
                        if robot:
                            rospy.loginfo(f"{robot} completed lap {self.lap_counts[tag_id]}")
                            self.lap_publishers[tag_id].publish(Float32(data=self.lap_counts[tag_id]))
                        last_lap_time[tag_id] = now_time

                self.last_positions[tag_id] = pos
                self.last_lap_time = last_lap_time

    def is_point_in_zone(self, point, zone):
        x, y = point
        return zone['x_min'] <= x <= zone['x_max'] and zone['y_min'] <= y <= zone['y_max']

    def tag_ground_point(self, det):
        return np.array(det.center, float)
    def apply_ignore_mask(self, gray):
        """Black out configured ignore_zones before tag detection."""
        if not getattr(self, "ignore_zones", None):
            return gray

        h, w = gray.shape[:2]
        for z in self.ignore_zones:
            x0 = int(z.get("x_min", 0))
            x1 = int(z.get("x_max", 0))
            y0 = int(z.get("y_min", 0))
            y1 = int(z.get("y_max", 0))

            # clip
            x0 = max(0, min(w, x0))
            x1 = max(0, min(w, x1))
            y0 = max(0, min(h, y0))
            y1 = max(0, min(h, y1))

            if x1 > x0 and y1 > y0:
                gray[y0:y1, x0:x1] = 0
        return gray
    def visualize_zones(self):
        img = self.first_image.copy()
        for i, zone in enumerate(self.merge_zones):
            cv2.rectangle(img,
                        (int(zone['x_min']), int(zone['y_min'])),
                        (int(zone['x_max']), int(zone['y_max'])),
                        (0, 0, 255), 2)
            cv2.putText(img, f"Merge Zone {i}",
                        (int(zone['x_min']), int(zone['y_min']) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        if self.fuel_zone:
            zone = self.fuel_zone
            cv2.rectangle(img,
                        (int(zone['x_min']), int(zone['y_min'])),
                        (int(zone['x_max']), int(zone['y_max'])),
                        (0, 255, 0), 2)
            cv2.putText(img, "Fuel Zone",
                        (int(zone['x_min']), int(zone['y_min']) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        if getattr(self, "charge_gate_zone", None):
            zone = self.charge_gate_zone
            cv2.rectangle(img,
                        (int(zone['x_min']), int(zone['y_min'])),
                        (int(zone['x_max']), int(zone['y_max'])),
                        (255, 255, 0), 2)
            cv2.putText(img, "Charge Gate",
                        (int(zone['x_min']), int(zone['y_min']) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        if self.has_finish and self.finish_line:
            x = self.finish_line.get('x', None)
            direction = self.finish_line.get('direction', '')
            if x is not None:
                if direction == "left_to_right":
                    cv2.line(img, (x, 0), (x, img.shape[0]), (0, 0, 255), 2)
                    cv2.putText(img, "Finish Line", (x + 5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                elif direction == "top_to_bottom":
                    cv2.line(img, (0, x), (img.shape[1], x), (0, 0, 255), 2)
                    cv2.putText(img, "Finish Line", (30, x + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        # Ignore zones (NEW)
        for z in getattr(self, "ignore_zones", []):
            cv2.rectangle(img,
                        (int(z['x_min']), int(z['y_min'])),
                        (int(z['x_max']), int(z['y_max'])),
                        (0, 0, 0), 2)
            cv2.putText(img, "Ignore",
                        (int(z['x_min']), int(z['y_min']) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        plt.figure("Zone Visualization")
        plt.imshow(img_rgb)
        plt.title(f"Zones for {self.cam_name}")
        plt.axis('off')
        plt.show(block=False)
        plt.pause(0.001)

if __name__ == '__main__':
    try:
        CamZoneManager(None, None)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
