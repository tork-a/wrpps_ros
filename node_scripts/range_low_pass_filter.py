#!/usr/bin/env python

import math

from sensor_msgs.msg import Range
import rospy


class RangeLowPassFilter(object):

    def __init__(self):
        # Default cut-off frequency is 9Hz
        self.input_coeff = rospy.get_param('~input_coeff', 0.825)
        self.reset_on_invalid_input = rospy.get_param('~reset_on_invalid_input', True)
        self.range_filtered = None

        self.pub = rospy.Publisher('~output', Range, queue_size=1)
        self.sub = rospy.Subscriber('~input', Range, self._cb)

    def _cb(self, msg):
        pub_msg = Range()
        if math.isnan(msg.range) or math.isinf(msg.range):
            if self.reset_on_invalid_input:
                self.range_filtered = None
            pub_msg.range = msg.range
        else:
            if self.range_filtered is None:
                self.range_filtered = msg.range
            self.range_filtered = self.input_coeff * msg.range \
                + (1 - self.input_coeff) * self.range_filtered
            pub_msg.range = self.range_filtered
        pub_msg.header = msg.header
        pub_msg.radiation_type = msg.radiation_type
        pub_msg.field_of_view = msg.field_of_view
        pub_msg.min_range = msg.min_range
        pub_msg.max_range = msg.max_range
        self.pub.publish(pub_msg)


if __name__ == '__main__':
    rospy.init_node('range_low_pass_filter')
    app = RangeLowPassFilter()
    rospy.spin()
