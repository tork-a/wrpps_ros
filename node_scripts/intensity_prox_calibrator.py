#!/usr/bin/env python

import math

from force_proximity_ros.msg import ProximityStamped
from sensor_msgs.msg import Range
from wrpps_ros.msg import IntensityModelParam
from std_srvs.srv import Trigger
from std_srvs.srv import TriggerResponse
import rospy


class IntensityProxCalibrator(object):

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
        self.i_height_from_tof = rospy.get_param('~i_height_from_tof', None)
        if self.i_height_from_tof is not None:
            self.i_height_from_tof = float(self.i_height_from_tof)
        self.i_queue_size_for_tof = rospy.get_param('~i_queue_size_for_tof', 2)
        self.tof_valid_min = rospy.get_param('~tof_valid_min', 0.04)
        self.tof_delay_from_i = rospy.get_param('~tof_delay_from_i', 0.0)
        self.tof_tm_tolerance = rospy.get_param('~tof_tm_tolerance', 0.02)
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
        self.tof_max_range = None
        self.i_raw = None
        self.i_diff_from_init = None
        self.tof_dist = None
        self.tof_tm = None
        self.i_diff_queue = []
        self.i_tm_queue = []
        self.is_latest_tof_published = True

        # Distance converted from intensity
        self.pub_range_i = rospy.Publisher(
            '~output/range_intensity', Range, queue_size=1)
        # Distance combined with ToF output
        ## Far part: ToF distance
        ## Middle & close part: distance generated from intensity
        self.pub_range_comb = rospy.Publisher(
            '~output/range_combined', Range, queue_size=1)
        # Parameters of intensity sensor model
        self.pub_param = rospy.Publisher(
            '~output/intensity_model_param', IntensityModelParam, queue_size=1)
        self.sub_input_i = rospy.Subscriber(
            '~input/intensity', ProximityStamped, self._intensity_cb)
        self.sub_input_tof = rospy.Subscriber(
            '~input/tof', Range, self._tof_cb)
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
            rospy.logwarn_throttle(10, 'Init prox is not set, so skipping')
            return
        self.i_diff_from_init = self.i_raw - self.i_init_value
        self.i_diff_queue.append(self.i_diff_from_init)
        self.i_tm_queue.append(msg.header.stamp)
        while len(self.i_diff_queue) > self.i_queue_size_for_tof:
            self.i_diff_queue.pop(0)
            self.i_tm_queue.pop(0)

        is_tof_valid = False
        if (self.tof_tm is not None) and (abs((self.tof_tm -
                                               msg.header.stamp).to_sec() -
                                              self.tof_delay_from_i) <=
                                          self.tof_tm_tolerance):
            is_tof_valid = True
        dist_combined = None
        if is_tof_valid:
            tof_d_from_i = self.tof_dist - self.i_height_from_tof
            if not self.is_latest_tof_published:
                dist_combined = tof_d_from_i
                self.is_latest_tof_published = True

        if self.i_refl_param is None:
            rospy.logwarn_throttle(10, 'Refl. param is not calculated yet')
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

            # Create distance combined with ToF output
            if distance == float('inf'):
                pass
            elif is_tof_valid and ((tof_d_from_i > self.i_valid_max_dist) and
                                   (self.i_diff_from_init <
                                    ((self.i_valid_min + self.i_valid_max) /
                                     2.0))):
                pass
            elif (not is_tof_valid) and (self.i_diff_from_init < self.i_valid_min):
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

        if dist_combined is not None:
            msg_range_comb = Range()
            msg_range_comb.header = msg.header
            msg_range_comb.range = dist_combined
            msg_range_comb.radiation_type = Range.INFRARED
            msg_range_comb.field_of_view = self.i_fov
            msg_range_comb.min_range = self.i_min_range
            msg_range_comb.max_range = self.tof_max_range
            self.pub_range_comb.publish(msg_range_comb)

    def _tof_cb(self, msg):
        self.tof_dist = msg.range
        self.tof_tm = msg.header.stamp
        self.tof_max_range = msg.max_range
        self.is_latest_tof_published = False
        if self.i_diff_from_init is None:
            rospy.logwarn_throttle(
                10, 'Prox diff_from_init is not set, so skipping')
            return
        if self.i_height_from_tof is None:
            self.i_height_from_tof = 0.0
        for i_diff, i_tm in zip(self.i_diff_queue, self.i_tm_queue):
            if abs((msg.header.stamp - i_tm).to_sec() -
                   self.tof_delay_from_i) > \
               self.tof_tm_tolerance:
                continue
            if i_diff < self.i_valid_min:
                continue
            if i_diff > self.i_valid_max:
                continue
            if self.tof_dist < self.tof_valid_min:
                continue
            if self.tof_dist > self.i_valid_max_dist + self.i_height_from_tof:
                continue
            self.i_refl_param = \
                i_diff * ((self.tof_dist - self.i_height_from_tof) ** 2)

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
    rospy.init_node('intensity_prox_calibrator')
    app = IntensityProxCalibrator()
    rospy.spin()
