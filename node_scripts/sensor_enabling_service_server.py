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

from std_msgs.msg import Bool
from std_msgs.msg import Empty
from std_srvs.srv import SetBool
from std_srvs.srv import SetBoolResponse
import rospy
import threading


class SensorEnablingServiceServer(object):

    """Wrap sensor enabling topic interfaces with service interfaces.
    Users should use service interfaces for their application.

    Details:
    Message published to the topic interface is sometimes lost in rosserial communication.
    The service interface can detect this and resend the message.
    Ideally, native service server of rosserial_arduino should work,
    but this causes "Lost sync with device, restarting..." when it is called repeatedly
    """

    def __init__(self):
        self.en_topics = rospy.get_param('~sensor_enabling_topics', [])
        self.got_en_topic = rospy.get_param(
            '~got_enabling_command_topic', 'got_enabling_command')
        self.res_timeout = rospy.get_param('~response_timeout', 0.1)
        self.is_retry = rospy.get_param('~retry_when_timed_out', True)
        self.is_got_en = False
        self.lock = threading.Lock()

        self.en_srvs = []
        self.en_pubs = []
        for en_t_i, en_t in enumerate(self.en_topics):
            self.en_srvs.append(
                rospy.Service(
                    en_t, SetBool, lambda req, idx=en_t_i: self._en_srv_cb(
                        idx, req)))
            self.en_pubs.append(rospy.Publisher(en_t, Bool, queue_size=1))

        self.got_en_sub = rospy.Subscriber(
            self.got_en_topic, Empty, self._got_en_cb)

    def _en_srv_cb(self, pub_idx, req):
        with self.lock:
            is_finished = False
            while not is_finished:
                self.is_got_en = False
                self.en_pubs[pub_idx].publish(Bool(data=req.data))
                start_tm = rospy.get_time()
                while True:
                    if self.is_got_en:
                        is_finished = True
                        break
                    if rospy.get_time() > (start_tm + self.res_timeout):
                        rospy.logwarn(
                            '[{}] {} does not respond'.format(
                                rospy.get_name(), self.en_topics[pub_idx]))
                        if not self.is_retry:
                            is_finished = True
                        else:
                            rospy.logwarn('[{}] retrying...'.format(rospy.get_name()))
                        break
                    rospy.sleep(0.001)
            return SetBoolResponse(success=self.is_got_en)

    def _got_en_cb(self, msg):
        self.is_got_en = True


if __name__ == '__main__':
    rospy.init_node('sensor_enabling_service_server')
    app = SensorEnablingServiceServer()
    rospy.spin()
