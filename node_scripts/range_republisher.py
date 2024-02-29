#!/usr/bin/env python

from sensor_msgs.msg import Range
from std_msgs.msg import Float32
import rospy


class RangeRepublisher(object):

    def __init__(self):
        self.frame_id = rospy.get_param('~frame_id', 'range_frame')
        # Default: VL53L0X
        self.fov = rospy.get_param('~field_of_view', 0.44)  # 0.44: 25 deg
        self.min_range = rospy.get_param('~min_range', 0.03)
        self.max_range = rospy.get_param('~max_range', 2.0)

        self.sub = rospy.Subscriber('~input', Float32, self._cb)
        self.pub = rospy.Publisher('~output', Range, queue_size=1)

    def _cb(self, in_msg):
        stamp = rospy.Time.now()  # Get timestamp ASAP after message arrival

        pub_msg = Range()
        pub_msg.header.stamp = stamp
        pub_msg.header.frame_id = self.frame_id
        pub_msg.field_of_view = self.fov
        pub_msg.min_range = self.min_range
        pub_msg.max_range = self.max_range
        pub_msg.radiation_type = Range.INFRARED
        pub_msg.range = in_msg.data
        self.pub.publish(pub_msg)


if __name__ == '__main__':
    rospy.init_node('range_republisher')
    app = RangeRepublisher()
    rospy.spin()
