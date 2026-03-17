#!/usr/bin/env python3
"""
Integration test for the tack node (ROS 2 version).

This test publishes a sailing_state message and checks that tack_rudder is
published in response.  It uses rclpy directly (no rostest/roslaunch needed)
and is run by pytest.

NOTE: This test requires a running ROS 2 daemon and the ``tack`` executable
to be available on PATH (i.e. the package must be installed/sourced first).
If rclpy is not available the test is skipped so that the plain unit-test
suite still passes in environments without ROS 2 installed.
"""

import subprocess
import sys
import time
import unittest

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Float32
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False


@unittest.skipUnless(ROS2_AVAILABLE, 'rclpy not available')
class TestTackNode(unittest.TestCase):
    """Test that the tack node responds to sailing_state messages."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_tack_node')
        cls.received = []

        cls.sub = cls.node.create_subscription(
            Float32, '/tack_rudder',
            lambda msg: cls.received.append(msg.data),
            10)

        cls.pub = cls.node.create_publisher(String, '/sailing_state', 10)

        # Give the tack node (started externally) time to come up
        time.sleep(0.2)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_tack_rudder_published(self):
        """Publishing switch_to_stbd_tack should produce a tack_rudder value."""
        msg = String()
        msg.data = 'switch_to_stbd_tack'
        deadline = time.time() + 2.0
        while time.time() < deadline:
            self.pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.received:
                break

        self.assertTrue(
            len(self.received) > 0,
            'No tack_rudder message received within 2 seconds')
        self.assertEqual(self.received[0], 90.0)


if __name__ == '__main__':
    unittest.main()

