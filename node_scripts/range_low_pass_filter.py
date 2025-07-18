#!/usr/bin/env python

# Software License Agreement (BSD 3-Clause License)
#
# Copyright (c) 2024- JSK Robotics Laboratory, Shun Hasegawa
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
