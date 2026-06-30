#!/usr/bin/env python3
import socket
import struct
import threading

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def recv_exact(conn, size):
    chunks = []
    remaining = size
    while remaining:
        chunk = conn.recv(remaining)
        if not chunk:
            raise ConnectionError("client disconnected")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


class TcpJpegImagePublisher:
    def __init__(self):
        rospy.init_node("tcp_jpeg_image_publisher")
        self.host = rospy.get_param("~host", "0.0.0.0")
        self.port = int(rospy.get_param("~port", 5010))
        self.topic = rospy.get_param("~topic", "image_raw")
        self.frame_id = rospy.get_param("~frame_id", "camera")
        self.bridge = CvBridge()
        self.pub = rospy.Publisher(self.topic, Image, queue_size=1)
        self.frames = 0

    def handle_client(self, conn, addr):
        rospy.loginfo("TCP JPEG client connected from %s:%s", addr[0], addr[1])
        try:
            while not rospy.is_shutdown():
                header = recv_exact(conn, 4)
                (size,) = struct.unpack("!I", header)
                if size <= 0 or size > 10_000_000:
                    raise ValueError(f"invalid jpeg payload size {size}")
                payload = recv_exact(conn, size)
                arr = np.frombuffer(payload, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is None:
                    rospy.logwarn("failed to decode JPEG payload of %d bytes", size)
                    continue
                msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = self.frame_id
                self.pub.publish(msg)
                self.frames += 1
        finally:
            conn.close()
            rospy.loginfo("TCP JPEG client disconnected")

    def run(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.host, self.port))
        srv.listen(1)
        srv.settimeout(1.0)
        rospy.loginfo("listening for TCP JPEG stream on %s:%d -> %s", self.host, self.port, self.topic)
        while not rospy.is_shutdown():
            try:
                conn, addr = srv.accept()
            except socket.timeout:
                continue
            thread = threading.Thread(target=self.handle_client, args=(conn, addr), daemon=True)
            thread.start()


if __name__ == "__main__":
    TcpJpegImagePublisher().run()
