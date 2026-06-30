#!/usr/bin/env python3

''' 
This script contains some hardcoded experimental configurations for traffic signal control.
In the paper 'A Macro-Micro Dimensionally Consistent Framework for Modeling Scaled Signalized Intersections',
we used these configurations to conduct exclusive experiments comparing the MiniCCAM platform and a SUMO baseline.

So far it fits a single-phase traffic signal control scenario with inflow control.
It can of course be modified to fit other experimental settings.

The mapping is based on:

    default_phase_group = {
        # Main Intersection, phase number 1x
        (300, 323): [12],
        (300, 319): [12],
        (300, 317): [12],
        (300, 310): [12],
        (300, 311): [12],
        (302, 315): [11,12], # right west to south
        (302, 319): [11],    # straight west to east
        (303, 319): None,    # reserved for future use
        (303, 317): [11],    # straight west to east
        (304, 323): [14],    # left west to north
        (305, 317): [13],
        (305, 319): [13],
        (305, 310): [13],
        (305, 311): [13],
        (305, 315): [13],
        (307, 323): [11,13], # right east to north
        (307, 310): [11],    # straight east to west
        (308, 311): [11],    # straight east to west
        (309, 315): [14],
        # West Intersection, phase number 2x
        (310, 325): [22],
        (310, 316): [23],
        (311, 316): [23],
        (312, 303): [22],
        (312, 304): [22],
        (312, 302): [22],
        (312, 316): [21,22],    # straight north to south
        (313, 325): [21],       # straight south to north
        (313, 302): [21],
        (313, 303): [21],
        (313, 304): [21],
        # South Intersection, phase number 3x
        (314, 313): [31],
        (314, 300): [31,32,33],
        (315, 313): [32,33,34],
        (315, 330): [33],
        (316, 330): [32,34],
        (316, 300): [34],
        # East intersection, phase number 4x
        (317, 333): [42],
        (318, 333): [41,43],
        (318, 309): [41],
        (318, 308): [41],
        (318, 307): [41],
        (319, 314): [41,42],
        (321, 314): [42,43],
        (321, 307): [42,43],
        (321, 308): [42,43],
        (321, 309): [43],
        # North intersection, phase number 5x
        (323, 321): [52,53,54],
        (323, 312): [53],
        (325, 321): [51],
        (325, 305): [51,53,54],
        (333, 312): [54],
        (333, 305): [52],

        # Special cases
        (330, 318): [0],
        (330, 331): [0], # special for exit the map
        (332, 318): [0], # special for entry the map
    }
'''

import rospy
from std_msgs.msg import Int32MultiArray, Int32, Bool


class traffic_signal_exp_exclusive:
    
    def __init__(self):
        # Test arm: North intersection, east→west straight (phase 54)
        self.test_phase_num             = 54
        self.test_cycyle_time_sec       = 77.78
        self.test_green_time_ratio      = 0.4   # fraction of cycle with green on test approach

        # Upstream inflow control: phase at East intersection feeding the test arm
        self.upperstream_phase_num      = 41
        self.upper_stream_timeheadway   = 10.37  # desired headway [s] between released vehicles

        # Keep always green for downstream phases
        # 0 is about entering/exiting the map
        self.downstream_phase_num_list = [0, 21, 32]
        
        self.signal_pub = rospy.Publisher('/green_phases', Int32MultiArray, queue_size=1)
    
        # Tag used as "upstream detector" (seen by robots as they approach from upstream)
        self.upper_stream_tag_id = 318

        # Store subscribers keyed by robot name
        self.tag_subscribers = {}

        # Inflow control state
        self.last_passage_time = None   # time when the last *released* vehicle was registered
        self.upper_stream_go   = False  # flag to open upstream phase for one release action
        self.car_waiting       = False  # we know a car is waiting upstream but headway not yet satisfied

        self.manual_override_test_phase = False
        self.manual_override_upper_phase = False

        self.override_sub_test = rospy.Subscriber(
            '/override_test_phase',
            Bool,
            self.override_test_phase_callback
        )
        self.override_sub_upper = rospy.Subscriber(
            '/override_upper_phase',
            Bool,
            self.override_upper_phase_callback
        )

        # Discover and subscribe to all current detected_tags topics
        self._initial_topic_scan()

        # Optionally, you can re-scan periodically if robots may start later:
        # rospy.Timer(rospy.Duration(2.0), self._periodic_topic_scan)

        # Timer for signal publishing (100 steps per cycle)
        self.elapsed_time = 0.0
        rospy.Timer(rospy.Duration(self.test_cycyle_time_sec / 100.0), self.timer_callback)

        rospy.loginfo("Traffic Signal Exclusive Experiment Node started.")

    # -------- Topic discovery / subscription ---------------------------------

    def _initial_topic_scan(self):
        """Scan ROS master once and subscribe to all */perception/detected_tags topics."""
        try:
            topics = rospy.get_published_topics()
        except rospy.ROSException as e:
            rospy.logwarn("Failed to get published topics during init: %s", str(e))
            return

        for topic, topic_type in topics:
            # Expecting: std_msgs/Int32 on /<robot_name>/perception/detected_tags
            if topic_type == 'std_msgs/Int32' and topic.endswith('/perception/detected_tag_id'):
                parts = topic.split('/')
                if len(parts) < 3:
                    continue
                robot_name = parts[1]
                if robot_name in self.tag_subscribers:
                    continue
                self.tag_subscribers[robot_name] = rospy.Subscriber(
                    topic,
                    Int32,
                    self.tag_callback,
                    callback_args=robot_name
                )
                rospy.loginfo("Subscribed to tag detection topic: %s for robot: %s", topic, robot_name)

    def _periodic_topic_scan(self):
        """Optional: rescan topics periodically to catch robots that start later."""
        try:
            topics = rospy.get_published_topics()
        except rospy.ROSException as e:
            rospy.logwarn("Failed to get published topics: %s", str(e))
            return

        for topic, topic_type in topics:
            if topic_type == 'std_msgs/Int32' and topic.endswith('/perception/detected_tag_id'):
                parts = topic.split('/')
                if len(parts) < 3:
                    continue
                robot_name = parts[1]
                if robot_name not in self.tag_subscribers:
                    self.tag_subscribers[robot_name] = rospy.Subscriber(
                        topic,
                        Int32,
                        self.tag_callback,
                        callback_args=robot_name
                    )
                    rospy.loginfo("Subscribed (periodic) to tag detection topic: %s for robot: %s", topic, robot_name)

    # -------- Tag callback: inflow detection & headway logic -----------------

    def tag_callback(self, msg, robot_name):
        """Called whenever any robot publishes a detected tag id."""
        detected_tag_id = msg.data

        if detected_tag_id != self.upper_stream_tag_id:
            return

        now = rospy.get_time()
        rospy.loginfo("Robot %s detected upper stream tag %d at t=%.2f",
                      robot_name, detected_tag_id, now)

        # First vehicle: release immediately
        if self.last_passage_time is None:
            self.last_passage_time = now
            self.upper_stream_go   = True     # tell timer to open upstream phase once
            self.car_waiting       = False
            rospy.loginfo("First upstream vehicle: allowing immediate release.")
            return

        time_since_last = now - self.last_passage_time

        if time_since_last >= self.upper_stream_timeheadway:
            # Headway already satisfied when tag is detected: release immediately
            self.last_passage_time = now
            self.upper_stream_go   = True
            self.car_waiting       = False
            rospy.loginfo(
                "Robot %s passed upstream tag with sufficient headway: %.2fs (>= %.2fs). Release now.",
                robot_name, time_since_last, self.upper_stream_timeheadway
            )
        else:
            # Too close to previous vehicle: hold this one at red until headway is satisfied
            self.upper_stream_go   = False
            self.car_waiting       = True
            rospy.loginfo(
                "Robot %s detected upstream tag but headway too short: %.2fs (< %.2fs). Holding at red.",
                robot_name, time_since_last, self.upper_stream_timeheadway
            )

    def override_test_phase_callback(self, msg):
        """Callback to manually override test phase green signal."""
        self.manual_override_test_phase = msg.data
        rospy.loginfo("Manual override for test phase set to: %s", str(msg.data))

    def override_upper_phase_callback(self, msg):
        """Callback to manually override upper stream phase green signal."""
        self.manual_override_upper_phase = msg.data
        rospy.loginfo("Manual override for upper stream phase set to: %s", str(msg.data))
    # -------- Timer: build and publish green phase set -----------------------

    def timer_callback(self, event):
        """
        Periodic callback:
        - runs a fixed-time cycle on the test phase (54),
        - enforces headway-based inflow on upstream phase (41),
        - keeps downstream phases always green.
        """
        now = rospy.get_time()

        # 1) Check if we have a waiting vehicle that can now be released
        if self.car_waiting and self.last_passage_time is not None:
            time_since_last = now - self.last_passage_time
            if time_since_last >= self.upper_stream_timeheadway:
                # Enough time has passed since the last released vehicle: open upstream now
                self.upper_stream_go = True
                self.car_waiting     = False
                self.last_passage_time = now  # update reference for next headway
                rospy.loginfo(
                    "Headway condition reached (%.2fs >= %.2fs). Releasing waiting vehicle upstream.",
                    time_since_last, self.upper_stream_timeheadway
                )

        # 2) Compute time within the signal cycle for the test phase
        step = self.test_cycyle_time_sec / 100.0
        self.elapsed_time += step
        t_in_cycle = self.elapsed_time % self.test_cycyle_time_sec

        green_phases = []

        # 3) Upstream inflow phase control: open once when upper_stream_go is set
        if self.upper_stream_go:
            green_phases.append(self.upperstream_phase_num)
            # We treat this as a one-shot release for this timer step
            self.upper_stream_go = False

        # 4) Test phase fixed-time control (single approach)
        if t_in_cycle < self.test_green_time_ratio * self.test_cycyle_time_sec:
            green_phases.append(self.test_phase_num)

        # 5) Always-green downstream phases
        green_phases.extend(self.downstream_phase_num_list)

        # 6) Publish green phase list
        msg = Int32MultiArray()
        msg.data = green_phases

        # force in  test phase if manual override is set true
        if self.manual_override_test_phase:
            if self.test_phase_num not in msg.data:
                msg.data.append(self.test_phase_num)
                rospy.loginfo("Manual override: forcing test phase %d green", self.test_phase_num)
        # force in upper stream phase if manual override is set true
        if self.manual_override_upper_phase:
            if self.upperstream_phase_num not in msg.data:
                msg.data.append(self.upperstream_phase_num)
                rospy.loginfo("Manual override: forcing upper stream phase %d green", self.upperstream_phase_num)
        self._periodic_topic_scan()
        self.signal_pub.publish(msg)

        rospy.logdebug("t_in_cycle=%.2f, green_phases=%s", t_in_cycle, green_phases)


def main():
    rospy.init_node('traffic_signal_exp_exclusive')
    node = traffic_signal_exp_exclusive()
    rospy.spin()


if __name__ == '__main__':
    main()
