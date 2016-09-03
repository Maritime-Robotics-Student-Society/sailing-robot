"""Code for staying near a target point (2016 station keeping challenge)"""
from LatLon import LatLon
from shapely.geometry import Point
import time

from .taskbase import TaskBase
from .heading_planning_laylines import HeadingPlan

class StationKeeping(TaskBase):
    def __init__(self, nav, marker_ll, linger=300, radius=5, wind_angle=75):
        """Machinery to stay near a given point.
        
        This is meant to be started when we're already close to the marker; we'll
        normally put it immediately after a to_waypoint task to go to the marker.

        nav : a Navigation object for common machinery.
        marker_ll : a (lat, lon) point marking where we'll try to stay close to.
        linger : time in seconds to stay there
        radius : distance in metres which we'll try to bounce around the marker
        wind_angle : the absolute wind angle to sail (in degrees) when inside
           radius. This will automatically be flipped according to the tack.
        """
        self.nav = nav
        self.marker_ll = marker_ll
        self.marker = Point(self.nav.latlon_to_utm(self.marker_ll))
        self.linger = linger
        self.radius = radius
        self.wind_angle = wind_angle
        self.goal_heading = 0
        self.sailing_state = 'normal'  # sailing state can be 'normal','tack_to_port_tack' or  'tack_to_stbd_tack'
        self.start_time = 0
        self.head_to_waypoint = HeadingPlan(nav, LatLon(**marker_ll),
                            target_radius=radius, tack_voting_radius=2*radius)

    debug_topics = [
        ('heading_to_waypoint', 'Float32'),
        ('distance_to_waypoint', 'Float32'),
        ('goal_wind_angle', 'Float32'),
    ]

    def start(self):
        self.start_time = time.time()

    def check_end_condition(self):
        "Are we done yet?"
        return time.time() - self.start_time > self.linger

    def calculate_state_and_goal(self):
        """Work out what we want the boat to do
        """
        dwp, hwp = self.nav.distance_and_heading(self.marker)
        if dwp > self.radius:
            return self.head_to_waypoint.calculate_state_and_goal()
        
        self.debug_pub('distance_to_waypoint', dwp)
        self.debug_pub('heading_to_waypoint', hwp)

        if self.nav.angle_to_wind() < 0:
            goal_wind_angle = -self.wind_angle
        else:
            goal_wind_angle = self.wind_angle
        
        self.debug_pub('goal_wind_angle', goal_wind_angle)
        return 'normal', self.nav.wind_angle_to_heading(goal_wind_angle)
