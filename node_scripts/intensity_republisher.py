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

from force_proximity_ros.msg import ProximityStamped
from std_msgs.msg import UInt32
import rospy


class IntensityRepublisher(object):

    def __init__(self):
        self.frame_id = rospy.get_param('~frame_id', 'intensity_frame')

        # Sensitivity of touch/release detection,
        # values closer to zero increase sensitivity
        self.sensitivity = 1000
        # exponential average weight parameter /
        # cut-off frequency for high-pass filter
        self.ea = 0.3
        # low-pass filtered intensity reading
        self.average_value = None
        # FA-II value
        self.fa2 = 0

        self.sub = rospy.Subscriber('~input', UInt32, self._cb)
        self.pub = rospy.Publisher(
            '~output', ProximityStamped, queue_size=1)

    def _cb(self, in_msg):
        stamp = rospy.Time.now()  # Get timestamp ASAP after message arrival

        pub_msg = ProximityStamped()
        pub_msg.proximity.proximity = in_msg.data
        if self.average_value is None:
            self.average_value = in_msg.data
        pub_msg.proximity.average = self.average_value
        fa2derivative = self.average_value - in_msg.data - self.fa2
        pub_msg.proximity.fa2derivative = fa2derivative
        self.fa2 = self.average_value - in_msg.data
        pub_msg.proximity.fa2 = self.fa2
        if self.fa2 < -self.sensitivity:
            pub_msg.proximity.mode = 'T'
        elif self.fa2 > self.sensitivity:
            pub_msg.proximity.mode = 'R'
        else:
            pub_msg.proximity.mode = '0'
        self.average_value = int(self.ea * in_msg.data +
                                 (1 - self.ea) * self.average_value)

        pub_msg.header.stamp = stamp
        pub_msg.header.frame_id = self.frame_id
        self.pub.publish(pub_msg)


if __name__ == '__main__':
    rospy.init_node('intensity_republisher')
    app = IntensityRepublisher()
    rospy.spin()
