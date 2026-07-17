#!/usr/bin/env python3
"""
Zone calibration snapshot tool.

Subscribes to a ROS image topic (while race-control is running), runs
AprilTag detection on one frame, draws configured zone boxes and actual
robot pixel positions, then saves an annotated PNG.

Usage (ROS must be running with cameras active):
  python3 zone_calibrate.py                  # cam2 (charging side)
  python3 zone_calibrate.py --cam1           # cam1
  python3 zone_calibrate.py --topic /usb_cam_2/image_raw --zones zones_cam2.yaml

Output: /tmp/zone_cal.png  (or /tmp/zone_cal_cam1.png for --cam1)
"""

import argparse, os, sys, threading

import cv2
import numpy as np
import yaml
import dt_apriltags
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

PKG    = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(PKG, "config")


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f)


def draw_box(img, zone, label, color):
    x0, x1 = int(zone["x_min"]), int(zone["x_max"])
    y0, y1 = int(zone["y_min"]), int(zone["y_max"])
    cv2.rectangle(img, (x0, y0), (x1, y1), color, 2)
    cv2.putText(img, label, (x0, max(y0 - 6, 12)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)


def annotate(img, zone_cfg, robot_tags, detections):
    out = img.copy()

    if zone_cfg.get("charge_gate_zone"):
        draw_box(out, zone_cfg["charge_gate_zone"], "CHARGE GATE", (0, 200, 255))
    if zone_cfg.get("fuel_zone"):
        draw_box(out, zone_cfg["fuel_zone"], "FUEL", (0, 200, 0))
    for i, mz in enumerate(zone_cfg.get("merge_zones", [])):
        draw_box(out, mz, f"MERGE {i}", (255, 80, 0))
    for iz in zone_cfg.get("ignore_zones", []):
        draw_box(out, iz, "ignore", (80, 80, 80))

    tag_to_robot = {v: k for k, v in robot_tags.items()}
    known_ids    = set(robot_tags.values())

    for det in detections:
        if det.tag_id not in known_ids:
            continue
        cx, cy = int(det.center[0]), int(det.center[1])
        robot  = tag_to_robot.get(det.tag_id, f"tag{det.tag_id}")
        cv2.drawMarker(out, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 28, 3)
        for corner in det.corners:
            cv2.circle(out, (int(corner[0]), int(corner[1])), 5, (0, 0, 200), -1)
        cv2.putText(out, f"{robot} ({cx},{cy})", (cx + 12, cy - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)

    leg = [
        ("CHARGE GATE zone", (0, 200, 255)),
        ("FUEL zone",        (0, 200, 0)),
        ("MERGE zone",       (255, 80, 0)),
        ("Robot position",   (0, 0, 255)),
    ]
    for i, (txt, col) in enumerate(leg):
        cv2.putText(out, txt, (10, 25 + i * 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, col, 2)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic",  default="/usb_cam_2/image_raw")
    ap.add_argument("--zones",  default=os.path.join(CONFIG, "zones_cam2.yaml"))
    ap.add_argument("--tags",   default=os.path.join(CONFIG, "tag_robot_map.yaml"))
    ap.add_argument("--out",    default="/tmp/zone_cal.png")
    ap.add_argument("--cam1",   action="store_true",
                    help="shortcut: use cam1 topic and zones_cam1.yaml")
    args = ap.parse_args()

    if args.cam1:
        args.topic = "/usb_cam_1/image_raw"
        args.zones = os.path.join(CONFIG, "zones_cam1.yaml")
        if args.out == "/tmp/zone_cal.png":
            args.out = "/tmp/zone_cal_cam1.png"

    zone_cfg   = load_yaml(args.zones)
    robot_tags = load_yaml(args.tags)

    detector = dt_apriltags.Detector(
        families="tag36h11", nthreads=2,
        quad_decimate=1.0, quad_sigma=0.0,
        refine_edges=True, decode_sharpening=0.25)

    bridge  = CvBridge()
    got     = threading.Event()
    frame_holder = [None]

    def cb(msg):
        if got.is_set():
            return
        try:
            frame_holder[0] = bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            try:
                frame_holder[0] = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            except Exception:
                return
        got.set()

    rospy.init_node("zone_calibrate", anonymous=True)
    sub = rospy.Subscriber(args.topic, Image, cb, queue_size=1)

    print(f"Waiting for a frame on {args.topic} …")
    if not got.wait(timeout=10.0):
        sys.exit(f"No image received on {args.topic} after 10 s — is ROS running?")

    sub.unregister()
    frame = frame_holder[0]

    gray       = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    detections = detector.detect(gray)

    known_ids  = set(robot_tags.values())
    robot_dets = [d for d in detections if d.tag_id in known_ids]

    print(f"\nDetected {len(detections)} total tags, {len(robot_dets)} robot(s):")
    tag_to_robot = {v: k for k, v in robot_tags.items()}
    for d in robot_dets:
        name = tag_to_robot.get(d.tag_id, "?")
        print(f"  {name} (tag {d.tag_id})  center=({int(d.center[0])}, {int(d.center[1])})")

    if not robot_dets:
        print("  [no robots visible — place robots in camera view and retry]")

    annotated = annotate(frame, zone_cfg, robot_tags, robot_dets)
    cv2.imwrite(args.out, annotated)
    print(f"\nSaved: {args.out}")
    print("Open that image to see robot positions vs zone boxes.")
    print(f"\nTip: edit {args.zones} to adjust x_min/x_max/y_min/y_max.")


if __name__ == "__main__":
    main()
