import unittest

from sailing_robot.navigation import Navigation, angle_average

class AngleAverageTests(unittest.TestCase):
    def test(self):
        angle_average([])

        angle_average([1,1,1])

        angle_average([1,359])

        angle_average([90,270])
