from pytest import approx
from sailing_robot.navigation import (angleAbsDistance, angle_subtract,
    Navigation,
)

def test_angle_abs_difference():
    assert angleAbsDistance(90, 50) == 40
    assert angleAbsDistance(50, 90) == 40
    assert angleAbsDistance(350, 40) == 50
    assert angleAbsDistance(40, 350) == 50

def test_angle_subtract():
    assert angle_subtract(40, 10) == 30
    assert angle_subtract(40, 90) == -50
    assert angle_subtract(10, 350) == 20
    assert angle_subtract(340, 10) == -30

def test_utm_latlon_conversion():
    n = Navigation(utm_zone=30)
    lat = 50.927482
    lon = -1.408787
    x, y = n.latlon_to_utm(lat, lon)
    ll = n.utm_to_latlon(x, y)
    assert ll.lat.decimal_degree == approx( lat)
    assert ll.lon.decimal_degree == approx( lon)

class DummyNSFMsg(object):
    def __init__(self, lat, lon):
        self.latitude = lat
        self.longitude = lon

def test_safety_zone():
    n = Navigation(safety_zone_ll=[
        (50.78, 1.00),
        (50.78, 1.04),
        (50.82, 1.04),
        (50.82, 1.00),
    ])
    n.update_position(DummyNSFMsg(50.8, 1.02))
    assert n.check_safety_zone() == 0
    
    n.update_position(DummyNSFMsg(50.78001, 1.02))
    assert n.check_safety_zone() == 1
    
    n.update_position(DummyNSFMsg(50.75, 1.02))
    assert n.check_safety_zone() == 2
