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
