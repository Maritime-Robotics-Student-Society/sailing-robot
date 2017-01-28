"""Code for staying near a target point (2016 station keeping challenge)"""
from LatLon import LatLon
from shapely.geometry import Point
import time

from .taskbase import TaskBase
from .heading_planning_laylines import HeadingPlan
from .navigation import angleAbsDistance

class ObstacleWaypoints(TaskBase):
    def __init__(self, nav, normal_wp_plan, obstacle_wp_plan):
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
        self.normal_wp_plan = normal_wp_plan
        self.obstacle_wp_plan = obstacle_wp_plan
        self.obstacle_detected = False

    debug_topics = [
        ('dbg_heading_to_waypoint', 'Float32'),
        ('dbg_distance_to_waypoint', 'Float32'),
        ('dbg_goal_wind_angle', 'Float32'),
    ]

    def receive_detection(self, msg):
        if msg.data == 'detected':
            # Check if we're facing in the direction of the waypoint
            dwp, hwp = self.nav.distance_and_heading(self.normal_wp_plan.waypoint_xy)
            if angleAbsDistance(hwp, self.nav.heading) < 30:
                self.obstacle_detected = True

    def init_ros(self):
        import rospy
        from std_msgs.msg import String
        rospy.Subscriber('camera_detection', String, self.receive_detection)

    def start(self):
        self.obstacle_detected = False

    @property
    def active_plan(self):
        return self.obstacle_wp_plan if self.obstacle_detected else self.normal_wp_plan

    def check_end_condition(self):
        "Are we done yet?"
        return self.active_plan.check_end_condition()

    def calculate_state_and_goal(self):
        """Work out what we want the boat to do
        """
        self.debug_pub('dbg_latest_waypoint_id', self.active_plan.waypoint_id)
        return self.active_plan.calculate_state_and_goal()
