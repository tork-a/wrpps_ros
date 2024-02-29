#!/usr/bin/env python

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
