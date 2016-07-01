import unittest

from sailing_robot.navigation import angle_average

class AngleAverageTests(unittest.TestCase):
    def test_realValues(self):
        self.assertAlmostEqual(angle_average([1,2,3,4,5]), 3)
        self.assertAlmostEqual(angle_average([10,90]), 50)
        self.assertAlmostEqual(angle_average([11,351]), 1)

    def test_extreme(self):
        angle_average([])

        angle_average([1,1,1])

        angle_average([1,359])

        angle_average([90,270])
