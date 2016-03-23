#!/usr/bin/env python

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
        msg = q.get(timeout=2)
        self.assertEqual(msg.data, 90)

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestOnShutdown, sys.argv)
