#!/usr/bin/env python3

import rospy
import yaml
from std_msgs.msg import Bool


class ZoneStateAggregator:
    def __init__(self):
        rospy.init_node("zone_state_aggregator")

        robot_tag_config_path = rospy.get_param("~robot_tag_config_path")
        with open(robot_tag_config_path, "r") as f:
            robot_tags = yaml.safe_load(f)

        self.camera_names = rospy.get_param("~camera_names", ["usb_cam_1", "usb_cam_2"])
        self.fuel_camera_names = rospy.get_param("~fuel_camera_names", ["usb_cam_2"])
        self.fuel_hold_sec = float(rospy.get_param("~fuel_hold_sec", 0.5))
        self.states = {}
        self.last_fuel_true = {}
        self.fuel_pubs = {}
        self.merge_pubs = {}
        self.charge_gate_pubs = {}
        self.tag_visible_pubs = {}

        for robot_name in robot_tags:
            self.states[robot_name] = {
                "fuel": {cam: False for cam in self.camera_names},
                "merge": {cam: False for cam in self.camera_names},
                "charge_gate": {cam: False for cam in self.camera_names},
                "tag_visible": {cam: False for cam in self.camera_names},
            }
            self.last_fuel_true[robot_name] = None
            self.fuel_pubs[robot_name] = rospy.Publisher(
                f"/{robot_name}/in_fuel_zone", Bool, queue_size=1
            )
            self.merge_pubs[robot_name] = rospy.Publisher(
                f"/{robot_name}/in_merge_zone", Bool, queue_size=1
            )
            self.charge_gate_pubs[robot_name] = rospy.Publisher(
                f"/{robot_name}/in_charge_gate_zone", Bool, queue_size=1
            )
            self.tag_visible_pubs[robot_name] = rospy.Publisher(
                f"/{robot_name}/tag_visible", Bool, queue_size=1
            )

            for cam in self.camera_names:
                rospy.Subscriber(
                    f"/{robot_name}/{cam}/in_fuel_zone",
                    Bool,
                    self._make_callback(robot_name, cam, "fuel"),
                )
                rospy.Subscriber(
                    f"/{robot_name}/{cam}/in_merge_zone",
                    Bool,
                    self._make_callback(robot_name, cam, "merge"),
                )
                rospy.Subscriber(
                    f"/{robot_name}/{cam}/in_charge_gate_zone",
                    Bool,
                    self._make_callback(robot_name, cam, "charge_gate"),
                )
                rospy.Subscriber(
                    f"/{robot_name}/{cam}/tag_visible",
                    Bool,
                    self._make_callback(robot_name, cam, "tag_visible"),
                )

        rospy.loginfo(
            "ZoneStateAggregator started for robots=%s cameras=%s fuel_cameras=%s",
            list(robot_tags.keys()),
            self.camera_names,
            self.fuel_camera_names,
        )

    def _make_callback(self, robot_name, camera_name, field):
        def callback(msg):
            self.states[robot_name][field][camera_name] = bool(msg.data)
            if field == "fuel":
                now = rospy.Time.now().to_sec()
                fuel_values = [
                    self.states[robot_name]["fuel"].get(cam, False)
                    for cam in self.fuel_camera_names
                ]
                if any(fuel_values):
                    self.last_fuel_true[robot_name] = now
                last_true = self.last_fuel_true[robot_name]
                value = last_true is not None and (now - last_true) <= self.fuel_hold_sec
                self.fuel_pubs[robot_name].publish(Bool(data=value))
            else:
                value = any(self.states[robot_name][field].values())
                if field == "merge":
                    self.merge_pubs[robot_name].publish(Bool(data=value))
                elif field == "charge_gate":
                    self.charge_gate_pubs[robot_name].publish(Bool(data=value))
                else:
                    self.tag_visible_pubs[robot_name].publish(Bool(data=value))

        return callback


if __name__ == "__main__":
    ZoneStateAggregator()
    rospy.spin()
