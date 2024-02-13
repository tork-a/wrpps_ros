#!/usr/bin/env python

from sensor_msgs.msg import Range
import rospy


class RangeLowPassFilter(object):

    def __init__(self):
        # Default cut-off frequency is 9Hz
        self.input_coeff = rospy.get_param('~input_coeff', 0.825)
        self.range_filtered = None

        self.pub = rospy.Publisher('~output', Range, queue_size=1)
        self.sub = rospy.Subscriber('~input', Range, self._cb)

    def _cb(self, msg):
        if self.range_filtered is None:
            self.range_filtered = msg.range
        self.range_filtered = self.input_coeff * msg.range \
            + (1 - self.input_coeff) * self.range_filtered
        pub_msg = Range()
        pub_msg.header = msg.header
        pub_msg.radiation_type = msg.radiation_type
        pub_msg.field_of_view = msg.field_of_view
        pub_msg.min_range = msg.min_range
        pub_msg.max_range = msg.max_range
        pub_msg.range = self.range_filtered
        self.pub.publish(pub_msg)


if __name__ == '__main__':
    rospy.init_node('range_low_pass_filter')
    app = RangeLowPassFilter()
    rospy.spin()
