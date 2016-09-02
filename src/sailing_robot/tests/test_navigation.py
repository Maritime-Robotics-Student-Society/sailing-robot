from nose.tools import assert_equal, assert_almost_equal
from sailing_robot.navigation import (angleAbsDistance, angle_subtract,
    Navigation,
)

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

def test_utm_latlon_conversion():
    n = Navigation(utm_zone=30)
    lat = 50.927482
    lon = -1.408787
    x, y = n.latlon_to_utm(lat, lon)
    ll = n.utm_to_latlon(x, y)
    assert_almost_equal(ll.lat.decimal_degree,  lat)
    assert_almost_equal(ll.lon.decimal_degree,  lon)
