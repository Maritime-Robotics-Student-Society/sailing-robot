from nose.tools import assert_equal
from sailing_robot.navigation import angleAbsDistance, angle_subtract

def test_angle_abs_difference():
    assert_equal(angleAbsDistance(90, 50), 40)
    assert_equal(angleAbsDistance(50, 90), 40)
    assert_equal(angleAbsDistance(350, 40), 50)
    assert_equal(angleAbsDistance(40, 350), 50)

def test_angle_subtract():
    assert_equal(angle_subtract(40, 10), 30)
    assert_equal(angle_subtract(40, 90), -50)
    assert_equal(angle_subtract(10, 350), 20)
    assert_equal(angle_subtract(340, 10), -30)
