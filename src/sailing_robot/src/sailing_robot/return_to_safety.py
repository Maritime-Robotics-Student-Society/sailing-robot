import math
from shapely.geometry import Point, Polygon

from .navigation import angleSum
from .taskbase import TaskBase

# For calculations, lay lines don't extend to infinity.
# This is in m; 10km should be plenty for our purposes.
LAYLINE_EXTENT = 10000

class ReturnToSafetyZone(TaskBase):
    def __init__(self, nav):
        """Sail towards the centroid of the safety zone.

        *nav* is a sailing_robot.navigation.Navigation instance.
        
        This should only be in use if a safety zone was set.
        
        The task ends when it gets back comfortably inside the safety zone (with
        a double margin), so it doesn't deal with getting close to a waypoint.
        
        Heading for the centroid assumes that the safety zone is a sensible
        convex shape (not something like a C shape).
        """
        self.nav = nav
        self.waypoint_xy = self.nav.safety_zone.centroid
        margin = self.nav.safety_zone_margin
        self.safety_zone_double_margin = self.nav.safety_zone.buffer(-2*margin)
        # sailing state can be 'normal','switch_to_port_tack' or  'switch_to_stbd_tack'
        self.sailing_state = 'normal'

    def start(self):
        pass

    def check_end_condition(self):
        """Are we safe yet?"""
        return self.nav.position_xy.within(self.safety_zone_double_margin)

    debug_topics = [
        ('dbg_heading_to_waypoint', 'Float32'),
        ('dbg_distance_to_waypoint', 'Float32'),
        ('dbg_goal_wind_angle', 'Float32'),
    ]

    def distance_heading_to_waypoint(self):
        """Calculate where we are relative to the waypoint, for debugging.
        """
        dx = self.waypoint_xy.x - self.nav.position_xy.x
        dy = self.waypoint_xy.y - self.nav.position_xy.y
        d = (dx**2 + dy**2) ** 0.5
        h = math.degrees(math.atan2(dx, dy)) % 360
        return d, h

    def calculate_state_and_goal(self):
        """Work out what we want the boat to do
        """
        dwp, hwp = self.distance_heading_to_waypoint()
        self.debug_pub('dbg_distance_to_waypoint', dwp)
        self.debug_pub('dbg_heading_to_waypoint', hwp)
        self.debug_pub('dbg_latest_waypoint_id', 'safety_zone_centroid')

        boat_wind_angle = self.nav.angle_to_wind()
        if self.sailing_state != 'normal':
            # A tack/jibe is in progress
            if self.sailing_state == 'switch_to_port_tack':
                goal_angle = self.nav.beating_angle
                continue_tack = boat_wind_angle < goal_angle or boat_wind_angle > 120
            else:  # 'switch_to_stbd_tack'
                goal_angle = -self.nav.beating_angle
                continue_tack = boat_wind_angle > goal_angle or boat_wind_angle < -120
            
            if continue_tack:
                self.debug_pub('dbg_goal_wind_angle', goal_angle)
                return self.sailing_state, self.nav.wind_angle_to_heading(goal_angle)
            else:
                # Tack completed
                self.log('info', 'Finished tack (%s)', self.sailing_state)
                self.sailing_state = 'normal'

        on_port_tack = boat_wind_angle > 0

        wp_wind_angle = self.nav.heading_to_wind_angle(hwp)
        
        # Detect if the waypoint is downwind, if so head directly to it
        if (wp_wind_angle % 360) > 90 and (wp_wind_angle % 360) < 270:
            goal_wind_angle = wp_wind_angle
            state = 'normal'
            return state, self.nav.wind_angle_to_heading(goal_wind_angle)

        # If wp_wind_angle and boat_wind_angle have the same sign, we're on the better tack already
        # Or if we're between the laylines; stick to our current tack for now
        if (wp_wind_angle * boat_wind_angle > 0) \
                or self.nav.position_xy.within(self.lay_triangle()):
            tack_now = False
        else:
            tack_now = True

        if tack_now:
            # Ready about!
            if on_port_tack:
                state = 'switch_to_stbd_tack'
                goal_wind_angle = -self.nav.beating_angle
            else:
                state = 'switch_to_port_tack'
                goal_wind_angle = self.nav.beating_angle
            self.sailing_state = state
        else:
            # Stay on our current tack
            if on_port_tack:
                goal_wind_angle = max(wp_wind_angle, self.nav.beating_angle)
            else:
                goal_wind_angle = min(wp_wind_angle, -self.nav.beating_angle)
            state = 'normal'

        self.debug_pub('dbg_goal_wind_angle', goal_wind_angle)
        return state, self.nav.wind_angle_to_heading(goal_wind_angle)

    def lay_triangle(self):
        """Calculate the lay lines for the current waypoint.
        
        This returns a shapely Polygon with the two lines extended to
        LAYLINE_EXTENT (10km).
        """
        downwind = angleSum(self.nav.absolute_wind_direction(), 180)
        x0, y0 = self.waypoint_xy.x, self.waypoint_xy.y
        l1 = math.radians(angleSum(downwind, -self.nav.beating_angle))
        x1 = x0 + (LAYLINE_EXTENT * math.sin(l1))
        y1 = y0 + (LAYLINE_EXTENT * math.cos(l1))
        l2 = math.radians(angleSum(downwind, self.nav.beating_angle))
        x2 = x0 + (LAYLINE_EXTENT * math.sin(l2))
        y2 = y0 + (LAYLINE_EXTENT * math.cos(l2))
        return Polygon([(x0, y0), (x1, y1), (x2, y2)])
