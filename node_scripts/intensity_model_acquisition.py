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

from force_proximity_ros.msg import ProximityStamped
from sensor_msgs.msg import Range
from wrpps_ros.msg import IntensityModelParam
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse
import rospy


class IntensityModelAcquisition(object):

    def __init__(self):
        self.i_refl_param = rospy.get_param('~i_reflectance_param', None)
        if self.i_refl_param is not None:
            self.i_refl_param = float(self.i_refl_param)
        self.i_init_value = rospy.get_param('~i_init_value', None)
        if self.i_init_value is not None:
            self.i_init_value = int(self.i_init_value)
        self.i_valid_min = rospy.get_param('~i_valid_min', 50)
        self.i_valid_max = rospy.get_param('~i_valid_max', 1000)
        self.i_valid_max_dist = rospy.get_param('~i_valid_max_dist', 0.06)
        self.i_height_from_rng = rospy.get_param('~i_height_from_rng', None)
        if self.i_height_from_rng is not None:
            self.i_height_from_rng = float(self.i_height_from_rng)
        self.i_queue_size_for_rng = rospy.get_param('~i_queue_size_for_rng', 2)
        self.rng_valid_min = rospy.get_param('~rng_valid_min', 0.04)
        self.rng_delay_from_i = rospy.get_param('~rng_delay_from_i', 0.0)
        self.rng_tm_tolerance = rospy.get_param('~rng_tm_tolerance', 0.02)
        self.use_i_average = rospy.get_param('~use_i_average', False)
        self.rubber_t = rospy.get_param('~rubber_thickness', None)
        if self.rubber_t is not None:
            self.rubber_t = float(self.rubber_t)
        # Default value: 25 degrees
        self.i_fov = rospy.get_param('~i_field_of_view', 0.44)
        # Default value: VCNL4040
        self.i_min_range = rospy.get_param('~i_min_range', 0.0)
        # Default value: VCNL4040
        self.i_max_range = rospy.get_param('~i_max_range', 0.2)
        self.rng_max_range = None
        self.i_raw = None
        self.i_diff_from_init = None
        self.rng_dist = None
        self.rng_tm = None
        self.i_diff_queue = []
        self.i_tm_queue = []
        self.is_latest_rng_published = True

        # Distance converted from intensity
        self.pub_range_i = rospy.Publisher(
            '~output/range_intensity', Range, queue_size=1)
        # Distance combined with range sensor
        ## Far part: range sensor distance
        ## Middle & close part: distance generated from intensity
        self.pub_range_comb = rospy.Publisher(
            '~output/range_combined', Range, queue_size=1)
        # Parameters of intensity sensor model
        self.pub_param = rospy.Publisher(
            '~output/intensity_model_param', IntensityModelParam, queue_size=1)
        self.sub_input_i = rospy.Subscriber(
            '~input/intensity', ProximityStamped, self._intensity_cb)
        self.sub_input_rng = rospy.Subscriber(
            '~input/range', Range, self._rng_cb)
        self.init_srv = rospy.Service(
            '~set_init_intensity', Trigger, self._set_init_intensity)
        self.reset_refl_param_srv = rospy.Service(
            '~reset_reflectance_param', Trigger, self._reset_refl_param)

    def _intensity_cb(self, msg):
        if self.use_i_average:
            self.i_raw = msg.proximity.average
        else:
            self.i_raw = msg.proximity.proximity
        if self.i_init_value is None:
            rospy.logwarn_throttle(
                10,
                '[{}] Init prox is not set, so skipping'.format(
                    rospy.get_name())
            )
            return
        self.i_diff_from_init = self.i_raw - self.i_init_value
        self.i_diff_queue.append(self.i_diff_from_init)
        self.i_tm_queue.append(msg.header.stamp)
        while len(self.i_diff_queue) > self.i_queue_size_for_rng:
            self.i_diff_queue.pop(0)
            self.i_tm_queue.pop(0)

        is_rng_valid = False
        if (self.rng_tm is not None) and (abs((self.rng_tm -
                                               msg.header.stamp).to_sec() -
                                              self.rng_delay_from_i) <=
                                          self.rng_tm_tolerance):
            is_rng_valid = True
        dist_combined = None
        if is_rng_valid:
            rng_d_from_i = self.rng_dist - self.i_height_from_rng
            if not self.is_latest_rng_published:
                dist_combined = rng_d_from_i
                self.is_latest_rng_published = True

        if self.i_refl_param is None:
            rospy.logwarn_throttle(
                10,
                '[{}] Refl. param is not calculated yet'.format(
                    rospy.get_name())
            )
        else:
            if self.i_diff_from_init > 0:
                diff_plus = self.i_diff_from_init
            else:
                diff_plus = float('inf')
            distance = math.sqrt(self.i_refl_param / diff_plus)
            if distance == 0:
                distance = float('inf')

            if (self.rubber_t is not None) and (distance < self.rubber_t):
                # If distance is under thickness
                init_refl = self.i_init_value * (self.rubber_t ** 2)
                distance = math.sqrt(
                    (self.i_refl_param + init_refl) / self.i_raw)

            # Create distance combined with range sensor
            if distance == float('inf'):
                pass
            elif is_rng_valid and ((rng_d_from_i > self.i_valid_max_dist) and
                                   (self.i_diff_from_init <
                                    ((self.i_valid_min + self.i_valid_max) /
                                     2.0))):
                pass
            elif not is_rng_valid:
                pass
            else:
                dist_combined = distance

            msg_range_i = Range()
            msg_range_i.header = msg.header
            msg_range_i.range = distance
            msg_range_i.radiation_type = Range.INFRARED
            msg_range_i.field_of_view = self.i_fov
            msg_range_i.min_range = self.i_min_range
            msg_range_i.max_range = self.i_max_range
            self.pub_range_i.publish(msg_range_i)

            msg_param = IntensityModelParam()
            msg_param.header = msg.header
            msg_param.reflectance_param = self.i_refl_param
            msg_param.init_value = self.i_init_value
            self.pub_param.publish(msg_param)

        msg_range_comb = Range()
        if dist_combined is not None:
            msg_range_comb.range = dist_combined
        else:
            msg_range_comb.range = float('nan')
        msg_range_comb.header = msg.header
        msg_range_comb.radiation_type = Range.INFRARED
        msg_range_comb.field_of_view = self.i_fov
        msg_range_comb.min_range = self.i_min_range
        if self.rng_max_range is not None:
            msg_range_comb.max_range = self.rng_max_range
        else:
            msg_range_comb.max_range = self.i_max_range
        self.pub_range_comb.publish(msg_range_comb)

        # We do not limit range values between min_range and max_range because
        # https://www.ros.org/reps/rep-0117.html#reference-implementation
        # accepts the case where the value is not between these limits
        # and we assume someone wants to know the value even in that case

    def _rng_cb(self, msg):
        self.rng_max_range = msg.max_range
        if math.isnan(msg.range) or math.isinf(msg.range):
            return
        self.rng_dist = msg.range
        self.rng_tm = msg.header.stamp
        self.is_latest_rng_published = False
        if self.i_diff_from_init is None:
            rospy.logwarn_throttle(
                10,
                '[{}] Prox diff_from_init is not set, so skipping'.format(
                    rospy.get_name())
            )
            return
        if self.i_height_from_rng is None:
            self.i_height_from_rng = 0.0
        for i_diff, i_tm in zip(self.i_diff_queue, self.i_tm_queue):
            if abs((msg.header.stamp - i_tm).to_sec() -
                   self.rng_delay_from_i) > \
               self.rng_tm_tolerance:
                continue
            if i_diff < self.i_valid_min:
                continue
            if i_diff > self.i_valid_max:
                continue
            if self.rng_dist < self.rng_valid_min:
                continue
            if self.rng_dist > self.i_valid_max_dist + self.i_height_from_rng:
                continue
            self.i_refl_param = \
                i_diff * ((self.rng_dist - self.i_height_from_rng) ** 2)

    def _set_init_intensity(self, req):
        is_success = True
        if self.i_raw is not None:
            self.i_init_value = self.i_raw
        else:
            is_success = False
        return TriggerResponse(success=is_success)

    def _reset_refl_param(self, req):
        self.i_refl_param = None
        return TriggerResponse(success=True)


if __name__ == '__main__':
    rospy.init_node('intensity_model_acquisition')
    app = IntensityModelAcquisition()
    rospy.spin()
