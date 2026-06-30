#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from pupil_apriltags import Detector
import cv2
from cv_bridge import CvBridge

robot_dict = {
    # 2: 'vivian',
    # 3: 'gina',
    4: 'lucas',
    # 5: 'daisy',
    # 6: 'henry',
    # 7: 'dorie',
    # 8: 'luna',
    10: 'fiona'
}

# what we do is that if a robot passes a virtual line in the image (vertically) already,
# we enforce a minimum *time headway* between successive crossings.
# when we see the next car crossing the line, we check the time since the last valid crossing:
#   - if it is smaller than the configured headway, we send '<robotname>/local_brake' = True
#     and keep it for the remaining time needed to reach the headway
#   - when that time is up, we send False to '<robotname>/local_brake'


# camera params (calibration in full resolution)
CAL_W, CAL_H = 1920, 1080
fx = 790.968410
fy = 786.233150
cx = 965.047938
cy = 552.795916


class VirtualStopper:

    def __init__(self):
        rospy.init_node('virtual_stopper', anonymous=True)

        self.at_deterctor = Detector(
            families='tag36h11',
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        self.image_sub = rospy.Subscriber('/top_cam.image_raw', Image, self.image_callback)
        self.bridge = CvBridge()

        # dynamically created publishers per robot
        self.robot_publishers = {}

        # per tag: last image x position and last seen time
        self.robot_last_seen = {}

        # minimum time headway between two crossings (seconds)
        # you can tune this via parameter server: ~min_headway
        self.min_headway = rospy.get_param('~min_headway', 10.37)

        # global last accepted "pass" time (for any robot)
        self.last_pass_time = None

        # virtual vertical line position in image coordinates (example value)
        # you can also expose this as a parameter if needed
        self.line_position = rospy.get_param('~line_position', 320)

    def get_publisher(self, robot_name):
        """Create or retrieve a Bool publisher for the robot's local_brake."""
        if robot_name not in self.robot_publishers:
            topic = f'/{robot_name}/local_brake'
            self.robot_publishers[robot_name] = rospy.Publisher(topic, Bool, queue_size=1)
        return self.robot_publishers[robot_name]

    def unbrake_robot(self, robot_name):
        pub = self.robot_publishers.get(robot_name)
        if pub is None:
            return
        pub.publish(Bool(data=False))
        rospy.loginfo(f'[virtual_stopper] Released brake for {robot_name}')

    def handle_crossing(self, tag_id, current_time):
        """Handle event that a particular tag has just crossed the virtual line."""
        robot_name = robot_dict[tag_id]
        pub = self.get_publisher(robot_name)

        # first robot: always allowed, ensure not braking
        if self.last_pass_time is None:
            pub.publish(Bool(data=False))
            self.last_pass_time = current_time
            rospy.loginfo(f'[virtual_stopper] {robot_name} is first to cross, no braking.')
            return

        dt = (current_time - self.last_pass_time).to_sec()

        # if headway is large enough, do nothing (and clear any residual brake)
        if dt >= self.min_headway:
            pub.publish(Bool(data=False))
            self.last_pass_time = current_time
            rospy.loginfo(
                f'[virtual_stopper] {robot_name} crossed with headway {dt:.2f}s '
                f'(>= {self.min_headway:.2f}s), no braking.'
            )
            return

        # too close in time: brake this robot for the remaining time
        remaining = self.min_headway - dt
        pub.publish(Bool(data=True))
        rospy.loginfo(
            f'[virtual_stopper] {robot_name} crossed too close to previous '
            f'({dt:.2f}s < {self.min_headway:.2f}s). Braking for {remaining:.2f}s.'
        )

        # schedule automatic release after remaining time
        rospy.Timer(
            rospy.Duration(remaining),
            lambda event, rn=robot_name: self.unbrake_robot(rn),
            oneshot=True
        )

        # update last_pass_time to reflect the enforced headway
        self.last_pass_time = self.last_pass_time + rospy.Duration(self.min_headway)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            rospy.logwarn(f'[virtual_stopper] cv_bridge error: {e}')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        tags = self.at_deterctor.detect(
            gray,
            estimate_tag_pose=False,
            camera_params=(fx, fy, cx, cy),
            tag_size=0.12
        )

        # filter tag to reasonable range, id has to be in robot_dict
        valid_tags = [tag for tag in tags if tag.tag_id in robot_dict.keys()]
        current_time = rospy.Time.now()

        # track which tags we see in this frame to clean up disappeared ones
        seen_ids = set()

        for tag in valid_tags:
            tag_id = tag.tag_id
            seen_ids.add(tag_id)

            # center x coordinate: average of opposite corners
            tag_center_x = int((tag.corners[0][0] + tag.corners[2][0]) / 2.0)

            # first time we see this tag: just record position
            if tag_id not in self.robot_last_seen:
                self.robot_last_seen[tag_id] = {'last_x': tag_center_x, 'last_time': current_time}
                continue

            last_x = self.robot_last_seen[tag_id]['last_x']

            # check if line has been crossed from left to right within this frame
            # you can add a reverse check if line direction can be swapped
            if last_x < self.line_position <= tag_center_x:
                self.handle_crossing(tag_id, current_time)

            # update last seen position and time
            self.robot_last_seen[tag_id] = {'last_x': tag_center_x, 'last_time': current_time}

        # remove tags that disappeared from view so that a robot can "rearm" the crossing logic
        for tag_id in list(self.robot_last_seen.keys()):
            if tag_id not in seen_ids:
                del self.robot_last_seen[tag_id]


if __name__ == '__main__':
    try:
        vs = VirtualStopper()
        rospy.loginfo('[virtual_stopper] Node started.')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
