#!/usr/bin/env python
#
# based on:http://docs.ros.org/diamondback/api/rospy_tutorials/html/test__on__shutdown_8py_source.html 
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id: test_peer_subscribe_notify.py 3803 2009-02-11 02:04:39Z rob_wheeler $

## Integration test for peer_subscribe_notify

PKG = 'rospy_tutorials'
NAME = 'peer_subscribe_notify_test'

import sys
import time
import unittest
from Queue import Queue

import rospy
import rostest
import roslib.scriptutil as scriptutil
from std_msgs.msg import String, Float32

def subscribe_queue(topic, msg_type):
    q = Queue()
    rospy.Subscriber(topic, msg_type, q.put)
    return q

class TestOnShutdown(unittest.TestCase):
    def test_notify(self):
        q = subscribe_queue("/tack_rudder", Float32)
        rospy.init_node(NAME, anonymous=True)
        p = rospy.Publisher("/sailing_state", String, queue_size=10)
        time.sleep(0.2) # Icky fudge factor to give the tack node time to be ready
        p.publish('switch_to_stbd_tack')
        msg = q.get(timeout=2)
        self.assertEqual(msg.data, 90)

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestOnShutdown, sys.argv)
