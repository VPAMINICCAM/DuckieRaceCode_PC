#!/usr/bin/python3
import rospy

from tof_drivers.tof400f_i2c import ToFVL53L1X
from sensor_msgs.msg import Range
from std_msgs.msg import Bool, Int32


TOF_STATUS_VALID = 9
TOF_STATUS_INVALID = 0


class ToFDriverNode:

    def __init__(self) -> None:
        
        rospy.on_shutdown(self.shut_hook)
        self.tof = ToFVL53L1X(address=0x29,bus_num=7)

        self.veh_name         = rospy.get_namespace().strip("/")
        if len(self.veh_name) == 0:
            self.veh_name = 'db19'
        self.tof_distance = 5
        self.tof_status = TOF_STATUS_INVALID

        # Keep every topic relative so the namespace supplied by the parent
        # robot launch resolves these to /<robot>/front_range{,_status}.
        self._pub_tof = rospy.Publisher('front_range', Range, queue_size=1)
        self._pub_tof_status = rospy.Publisher(
            'front_range_status', Int32, queue_size=1)
        # Compatibility for older consumers while deployments migrate.
        self._legacy_pub_tof = rospy.Publisher(
            'tof_distance', Range, queue_size=1)
        self._timer       = rospy.Timer(rospy.Duration(1/10),self._read_data)
        self._publish_res = rospy.Timer(rospy.Duration(1/10),self._publish_data)

        

        rospy.loginfo("%s: tof sensor ready",self.veh_name)
    
        rospy.Subscriber("robot_interface_shutdown", Bool, self.signal_shut)

    def signal_shut(self,msg:Bool):
        if msg.data:
            rospy.signal_shutdown('tof sensor node shutdown')

    def _read_data(self,_):

        value = self.tof.get_distance()
        
        if value is not None and value != -1:
            self.tof_distance = value
            self.tof_status = TOF_STATUS_VALID
        else:
            self.tof_status = TOF_STATUS_INVALID
        
    def _publish_data(self,_):

        r = Range()

        r.header.stamp      = rospy.Time.now()
        r.header.frame_id   = '/tof_sensor'
        r.radiation_type    = Range.INFRARED
        r.field_of_view     = (15 / 180) * 3.14
        r.min_range         = 0.05
        r.max_range         = 4
        # REP-117 permits +Inf for a clear/invalid return.  This prevents a
        # stale obstacle distance from looking like a fresh valid sample to
        # consumers that do not subscribe to front_range_status.
        r.range = (
            self.tof_distance
            if self.tof_status == TOF_STATUS_VALID
            else float('inf')
        )

        self._pub_tof.publish(r)
        self._pub_tof_status.publish(Int32(data=self.tof_status))
        self._legacy_pub_tof.publish(r)
    
    def shut_hook(self):
        self.tof.stop_sensor()
    
if __name__ == '__main__':

    try:
        rospy.init_node("tof_sensor")
        N = ToFDriverNode()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo('Keyboard Shutdown')
