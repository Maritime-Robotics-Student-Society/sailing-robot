import unittest

from LatLon import LatLon
from .heading_planning import HeadingPlan

class HeadingPlanTests(unittest.TestCase):
    def setUp(self):
        self.hp = HeadingPlan(beating_angle=45, tack_line_offset=0.01)
        self.hp.position = LatLon(50.7, -1.02)
        self.hp.waypoint = LatLon(50.7, -0.8)  # Head east

    def test_complete_tack_to_port(self):
        self.hp.sailing_state = 'tack_to_port_tack'
        self.hp.wind_direction = 0

        self.hp.heading = -50
        state, goal = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'tack_to_port_tack')
        self.assertEqual(goal, 45)

        self.hp.heading = 10
        state, goal = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'tack_to_port_tack')
        self.assertEqual(goal, 45)

        self.hp.heading = 47
        state, goal = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'normal')
        self.assertGreater(goal, 45)

    def test_continue_tack_to_stbd(self):
        self.hp.sailing_state = 'tack_to_stbd_tack'
        self.hp.wind_direction = 180  # Wind from the south

        self.hp.heading = 200
        state, goal = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'tack_to_stbd_tack')
        self.assertEqual(goal, 135)

        self.hp.heading = 170
        state, goal = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'tack_to_stbd_tack')
        self.assertEqual(goal, 135)

        self.hp.heading = 132
        state, goal = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'normal')
        self.assertLess(goal, 135)

    def test_plain_sailing(self):
        self.hp.wind_direction = 10
        self.hp.heading = 110
        state, goal = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'normal')
        # Heading ~= 90, but not exactly, because LatLon calculations are
        # not on a plane.
        self.assertGreater(goal, 89)
        self.assertLess(goal, 91)

    def test_tack_to_port(self):
        self.hp.wind_direction = 10
        self.hp.heading = 280  # We're reaching the wrong way!
        state, goal = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'tack_to_port_tack')

    def test_tack_to_stbd(self):
        self.hp.wind_direction = 190
        self.hp.heading = 280  # We're reaching the wrong way!
        state, goal = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'tack_to_stbd_tack')

    def test_to_windward_port_tack(self):
        self.hp.wind_direction = 90
        self.hp.heading = 180
        state, goal = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'normal')
        self.assertEqual(goal, 135)

        # Time to switch tack
        self.hp.position = LatLon(50.68, -1.018)
        state, goal = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'tack_to_stbd_tack')

    def test_to_windward_stbd_tack(self):
        self.hp.wind_direction = 90
        self.hp.heading = 0
        state, goal = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'normal')
        self.assertEqual(goal, 45)

        # Time to switch tack
        self.hp.position = LatLon(50.72, -1.018)
        state, goal = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'tack_to_port_tack')
