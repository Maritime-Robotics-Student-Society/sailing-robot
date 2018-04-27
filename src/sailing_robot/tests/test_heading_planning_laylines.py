import unittest
from nose.tools import assert_equal

from shapely.geometry import Point
from sailing_robot.heading_planning_laylines import HeadingPlan, LAYLINE_EXTENT
from sailing_robot.navigation import Navigation

class HeadingPlanTests(unittest.TestCase):
    def setUp(self):
        nav = Navigation(beating_angle=45, utm_zone=30)
        nav.heading = 0
        nav.wind_direction = 225
        self.hp = HeadingPlan(nav)
        self.hp.waypoint_xy = Point(0, 0)

    def test_lay_triangle(self):
        lt = self.hp.lay_triangle()
        coords = list(lt.exterior.coords)
        self.assertEqual(coords[0], (0, 0))

        self.assertAlmostEqual(coords[1][0], 0)
        self.assertAlmostEqual(coords[1][1], LAYLINE_EXTENT)

        self.assertAlmostEqual(coords[2][0], LAYLINE_EXTENT)
        self.assertAlmostEqual(coords[2][1], 0)

    def test_calculate(self):
        # Between the laylines
        self.hp.nav.position_xy = Point(50, 50)
        state, goal_heading = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'normal')
        self.assertAlmostEqual(goal_heading, 270)

        # Outside laylines, going in right direction
        self.hp.nav.position_xy = Point(50, -50)
        state, goal_heading = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'normal')
        self.assertAlmostEqual(goal_heading, 315)

        # Outside laylines, going in wrong direction
        self.hp.nav.position_xy = Point(-50, 50)
        state, goal_heading = self.hp.calculate_state_and_goal()
        self.assertEqual(state, 'switch_to_stbd_tack')
        self.assertAlmostEqual(goal_heading, 180)
