from collections import deque
import LatLon as ll
import math
from shapely.geometry import Point
import rospy

from .navigation import Navigation, angleSum, angleAbsDistance

class HeadingPlan:
    def __init__(self, nav, tack_line_offset=0.01,
            waypoint=ll.LatLon(50.742810, 1.014469), # somewhere in the solent
            target_radius=2,
            tack_decision_samples=100, tack_decision_threshold=0.75,
            ):
        """Heading planning machinery.

        beating_angle is the closest angle we can sail to the wind -
        the total dead zone is twice this angle. Measured in degrees.

        tack_line_offset is half the width of the tacking corridor we use when
        sailing towards a marker upwind, measured in km.
        
        utm_zone is the zone number of the UTM system to use. Southampton is in
        zone 30, Portugal in zone 29. http://www.dmap.co.uk/utmworld.htm
        Distance calculations will be less accurate the further from the
        specified zone you are.
        """
        self.nav = nav
        self.waypoint = waypoint
        x, y = self.nav.latlon_to_utm(waypoint.lat.decimal_degree, waypoint.lon.decimal_degree)
        self.waypoint_xy = Point(x, y)
        self.target_area = self.waypoint_xy.buffer(target_radius)
        self.tack_line_offset = tack_line_offset
        self.wp_heading = 0
        self.side_heading = 0
        self.alternate_heading = 0
        self.sailing_state = 'normal'  # sailing state can be 'normal','tack_to_port_tack' or  'tack_to_stbd_tack'
        self.tack_decision_samples = tack_decision_samples
        self.tack_decision_min = int(tack_decision_threshold * tack_decision_samples)
        self.tack_wanted = deque(maxlen=tack_decision_samples)
        self.tack_wanted_sum = 0
    
    def start(self):
        pass

    def check_end_condition(self):
        return self.nav.position_xy.within(self.target_area)

    def distance_heading_to_waypoint(self):
        dx = self.waypoint_xy.x - self.nav.position_xy.x
        dy = self.waypoint_xy.y - self.nav.position_xy.y
        d = (dx**2 + dy**2) ** 0.5
        h = math.degrees(math.atan2(dx, dy)) % 360
        return d, h

    def calculate_state_and_goal(self):
        """Work out what we want the boat to do
        """
        boat_wind_angle = self.nav.angle_to_wind()
        if self.sailing_state != 'normal':
            # A tack is in progress
            if self.sailing_state == 'tack_to_port_tack':
                beating_angle = self.nav.beating_angle
                continue_tack = boat_wind_angle < beating_angle
            else:  # 'tack_to_stbd_tack'
                beating_angle = -self.nav.beating_angle
                continue_tack = boat_wind_angle > beating_angle
            if continue_tack:
                return self.sailing_state, self.nav.wind_angle_to_heading(beating_angle)
            else:
                # Tack completed
                self.sailing_state = 'normal'
                rospy.logerr("Completed tack")
                self.tack_wanted.clear()
                self.tack_wanted_sum = 0

        wp_heading = self.nav.position_ll.heading_initial(self.waypoint)
        wp_wind_angle = self.nav.heading_to_wind_angle(wp_heading)
        want_tack_now = 0
        if (wp_wind_angle * boat_wind_angle) < 0:
            # These two have different signs, so we want the other tack
            want_tack_now = 1
        
        rospy.logwarn('Want tack now: %d' % want_tack_now)
        rospy.logwarn('Want tack sum: %d' % self.tack_wanted_sum)
        rospy.logwarn('Tack decision min: %d' % self.tack_wanted_sum)

        self.tack_wanted_sum += want_tack_now
        if self.tack_wanted_sum > self.tack_decision_min:
            # We have reached the threshold to trigger a tack
            if boat_wind_angle > 0:
                # On the port tack
                beating_angle = -self.nav.beating_angle
                tack_to = 'tack_to_stbd_tack'
            else:
                beating_angle = self.nav.beating_angle
                tack_to = 'tack_to_port_tack'
            self.sailing_state = tack_to
            return tack_to, self.nav.wind_angle_to_heading(beating_angle)

        # Update the rolling poll of whether we want to tack.
        if len(self.tack_wanted) >= self.tack_decision_samples:
            self.tack_wanted_sum -= self.tack_wanted.popleft()
        self.tack_wanted.append(want_tack_now)

        # Continue on our current tack
        goal_wind_angle = wp_wind_angle
        if boat_wind_angle > 0:
            # On the port tack
            goal_wind_angle = max(goal_wind_angle, self.nav.beating_angle)
        else:
            # On the starboard tack
            goal_wind_angle = min(goal_wind_angle, -self.nav.beating_angle)

        return 'normal', self.nav.wind_angle_to_heading(goal_wind_angle)
