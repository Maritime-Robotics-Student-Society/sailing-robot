#!/usr/bin/env python

PKG = 'rospy_tutorials'
NAME = 'peer_subscribe_notify_test'

import sys
import time
import unittest

import rospy
import rostest
import roslib.scriptutil as scriptutil
from std_msgs.msg import String, Float32


class TestOnShutdown(unittest.TestCase):
    def __init__(self, *args):
        super(TestOnShutdown, self).__init__(*args)
        self.success = False

    def callback(self, msg):
        print(rospy.get_caller_id(), "I heard %s" % msg.data)
        #greetings is only sent over peer_publish callback, so hearing it is a success condition
        if msg.data == 90:
            self.success = True

    def test_notify(self):
        rospy.Subscriber("/tack_rudder", Float32, self.callback)
        rospy.init_node(NAME, anonymous=True)
        p = rospy.Publisher("/sailing_state", String, queue_size=10)
        p.publish('tack_to_stbd_tack')
        timeout_t = time.time() + 10.0*1000 #10 seconds
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            time.sleep(0.1)
        self.assert_(self.success, str(self.success))

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestOnShutdown, sys.argv)
