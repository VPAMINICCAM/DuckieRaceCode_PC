#!/usr/bin/python3

import rospy

from tof_drivers.tof400f_usart import ToF400F

from sensor_msgs.msg import Range
from std_msgs.msg import Int32


TOF_STATUS_VALID = 9
TOF_STATUS_INVALID = 0

class ToFDriverNode:

    def __init__(self) -> None:
        
        rospy.on_shutdown(self.shut_hook)
        self.tof = ToF400F()

        self.veh_name         = rospy.get_namespace().strip("/")
        if len(self.veh_name) == 0:
            self.veh_name = 'db19'

        self.tof_distance = 5
        self.tof_status = TOF_STATUS_INVALID

        # Relative names inherit /<robot> from duckierace_start.launch.
        self._pub_tof = rospy.Publisher('front_range', Range, queue_size=1)
        self._pub_tof_status = rospy.Publisher(
            'front_range_status', Int32, queue_size=1)
        self._legacy_pub_tof = rospy.Publisher(
            'tof_distance', Range, queue_size=1)

        # Timers are created last so an early callback cannot observe missing
        # publishers or state.
        self._timer       = rospy.Timer(rospy.Duration(1/10),self._read_data)
        self._publish_res = rospy.Timer(rospy.Duration(1/5),self._publish_data)

        rospy.loginfo("%s: tof sensor ready",self.veh_name)

    def _read_data(self,_):

        value = self.tof.get_distance()
        
        if value is not None and value != -1:
            self.tof_distance = value * 0.01 # convert to meters
            if self.tof_distance > 1.36:
                self.tof_distance = 1.36
            self.tof_status = TOF_STATUS_VALID
        else:
            self.tof_status = TOF_STATUS_INVALID
        
    def _publish_data(self,_):

        r = Range()

        r.header.stamp      = rospy.Time.now()
        r.header.frame_id   = '/tof_sensor'
        r.radiation_type    = Range.INFRARED
        r.field_of_view     = (27 / 180) * 3.14
        r.min_range         = 0.05
        r.max_range         = 1.36
        r.range = (
            self.tof_distance
            if self.tof_status == TOF_STATUS_VALID
            else float('inf')
        )

        self._pub_tof.publish(r)
        self._pub_tof_status.publish(Int32(data=self.tof_status))
        self._legacy_pub_tof.publish(r)
    
    def shut_hook(self):
        del self.tof.serial

if __name__ == '__main__':

    try:
        rospy.init_node("tof_sensor")
        N = ToFDriverNode()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Keyboard Shutdown')
