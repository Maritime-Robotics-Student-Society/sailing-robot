"""Common navigation machinery used by different modules"""

import math
from LatLon import LatLon
from pyproj import Proj
from shapely.geometry import Point, Polygon

class Navigation(object):
    """Common navigation machinery used by different modules.
    
    Stores boat position (both lat/long and x/y based on UTM projection), and
    heading, along with apparent wind angle.
    """
    def __init__(self, 
                beating_angle=45, utm_zone=30, 
                safety_zone_ll=None, safety_zone_margin=5):
        """
        beating_angle : Closest absolute angle relative to the wind that we can
            sail
        utm_zone : Zone number of the UTM system to use. Southampton is in
            zone 30, Portugal in zone 29. http://www.dmap.co.uk/utmworld.htm
            Distance calculations will be less accurate the further from the
            specified zone you are.
        safety_zone_ll : A series of lat/lon points we should stay within.
        safety_zone_margin : The safety buffer (in metres) to stay inside
            the bounding box.
        """
        self.projection = Proj(proj='utm', zone=utm_zone, ellps='WGS84')
        self.position_ll = ll = LatLon(50.8, 1.02)
        x, y = self.latlon_to_utm(ll.lat.decimal_degree, ll.lon.decimal_degree)
        self.position_xy = Point(x, y)
        self.heading = 0.
        self.wind_direction = 0.
        self.beating_angle = beating_angle
        self.safety_zone_ll = safety_zone_ll
        self.safety_zone_margin = safety_zone_margin
        if safety_zone_ll:
            self.safety_zone = Polygon([
                self.latlon_to_utm(*p) for p in safety_zone_ll
            ])
            self.safety_zone_inner = self.safety_zone.buffer(-safety_zone_margin)
        else:
            self.safety_zone = self.safety_zone_inner = None
    
    def update_position(self, msg):
        self.position_ll = LatLon(msg.latitude, msg.longitude)
        x, y = self.latlon_to_utm(msg.latitude, msg.longitude)
        self.position_xy = Point(x, y)

    def latlon_to_utm(self, lat, lon):
        """Returns (x, y) coordinates in metres"""
        return self.projection(lon, lat)
    
    def utm_to_latlon(self, x, y):
        """Returns a LatLon object"""
        lon, lat = self.projection(x, y, inverse=True)
        return LatLon(lat, lon)

    def update_heading(self, msg):
        self.heading = msg.data
    
    def update_wind_direction(self, msg):
        self.wind_direction = msg.data
    
    def absolute_wind_direction(self):
        """Convert apparent wind direction to absolute wind direction"""
        # This assumes that our speed is negligible relative to wind speed.
        return angleSum(self.heading, self.wind_direction)

    def angle_to_wind(self):
        """Calculate angle relative to wind (-180 to 180)

        Angle relative to wind is reversed from wind direction: if the wind is
        coming from 90, the angle relative to the wind is -90.
        """
        wd = self.wind_direction
        if wd > 180:
            wd -= 360
        return -wd

    def heading_to_wind_angle(self, heading):
        """Convert a compass heading (0-360) to an angle relative to the wind (+-180)
        """
        return angle_subtract(heading, self.absolute_wind_direction())

    def wind_angle_to_heading(self, wind_angle):
        """Convert angle relative to the wind (+-180) to a compass heading (0-360).
        """
        return angleSum(self.absolute_wind_direction(), wind_angle)

    def check_safety_zone(self):
        """Check if the boat is within the safety zone.

        0 : Comfortably inside the safety zone (or no safety zone specified)
        1 : Inside the safety zone, but in the margin
        2 : Outside the safety zone
        """
        if self.safety_zone is None:
            return 0

        if self.position_xy.within(self.safety_zone_inner):
            return 0
        if self.position_xy.within(self.safety_zone):
            return 1
        return 2

    def distance_and_heading(self, wp):
        """Calculate the distance and heading from current position to wp.

        wp should both be a shapely.geometry.Point object
        """
        dx = wp.x - self.position_xy.x
        dy = wp.y - self.position_xy.y
        d = (dx**2 + dy**2) ** 0.5
        h = math.degrees(math.atan2(dx, dy)) % 360
        return d, h

################
# General utility functions
################

def angleSum(a,b):
    """Add two angles in degrees, returning a value mod 360
    """
    return (a+b)%360

def angleAbsDistance(a,b):
    """Magnitude of the difference between two angles.
    
    Result should always be between 0 and 180.
    """
    distanceA = abs((a - b) % 360)
    distanceB = abs((b - a) % 360)
    return min(distanceA, distanceB)

def angle_subtract(a, b):
    """Find the difference between two angles.
    
    The result should be between -180 (if a<b) and +180 (if a>b)
    """
    res = (a - b) % 360
    if res > 180:
        res -= 360
    return res

def angle_average(angle_list):
    """Compute the average angle of a list of angles (the result is % 360)
    """
    return math.degrees(math.atan2(sum([ math.sin(math.radians(x)) for x in angle_list]),
                                   sum([ math.cos(math.radians(x)) for x in angle_list]))) % 360
