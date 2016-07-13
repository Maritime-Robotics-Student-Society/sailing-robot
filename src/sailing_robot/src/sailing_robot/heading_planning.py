from collections import deque
import LatLon as ll
import math
from shapely.geometry import Point

from .taskbase import TaskBase

class TackVoting(object):
    def __init__(self, nsamples, threshold):
        self.nsamples = nsamples
        self.threshold = threshold
        self.votes = deque(maxlen=nsamples)
        # Start off indecisive, with equal votes either way
        self.votes.extend([1, 0] * (nsamples // 2))
        self.votes_sum = nsamples // 2

    def vote(self, value):
        # 0: Want starboard tack
        # 1: Want port tack
        if len(self.votes) >= self.nsamples:
            self.votes_sum -= self.votes.popleft()
        self.votes.append(value)
        self.votes_sum += value


    def tack_now(self, current_tack):
        # 0: Currently on starboard tack
        # 1: Currently on port tack
        if current_tack:
            # Port: tack to starboard?
            return self.votes_sum < (self.nsamples - self.threshold)
        else:
            # Starboard: tack to port?
            return self.votes_sum > self.threshold

    def reset(self, current_tack):
        # 0: Reached starboard tack
        # 1: Reached port tack
        if current_tack:
            self.votes.extend([1] * self.nsamples)
            self.votes_sum = self.nsamples
        else:
            self.votes.clear()
            self.votes_sum = 0

class HeadingPlan(TaskBase):
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
        self.tack_voting = TackVoting(tack_decision_samples,
                         int(tack_decision_threshold * tack_decision_samples))
    
    def start(self):
        pass

    def check_end_condition(self):
        return self.nav.position_xy.within(self.target_area)

    debug_topics = [
        ('heading_to_waypoint', 'Float32'),
        ('distance_to_waypoint', 'Float32'),
        ('goal_wind_angle', 'Float32'),
    ]

    def distance_heading_to_waypoint(self):
        dx = self.waypoint_xy.x - self.nav.position_xy.x
        dy = self.waypoint_xy.y - self.nav.position_xy.y
        d = (dx**2 + dy**2) ** 0.5
        h = math.degrees(math.atan2(dx, dy)) % 360
        return d, h

    def calculate_state_and_goal(self):
        """Work out what we want the boat to do
        """
        dwp, hwp = self.distance_heading_to_waypoint()
        self.debug_pub('distance_to_waypoint', dwp)
        self.debug_pub('heading_to_waypoint', hwp)

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
                self.log('info', 'Finished tack (%s)', self.sailing_state)
                self.tack_voting.reset(boat_wind_angle > 0)
                self.sailing_state = 'normal'

        on_port_tack = boat_wind_angle > 0

        wp_heading = self.nav.position_ll.heading_initial(self.waypoint)
        wp_wind_angle = self.nav.heading_to_wind_angle(wp_heading)
        # Tack voting: 0 for starboard tack, 1 for port tack
        current_tack_vote = int(wp_wind_angle > 0)
        self.tack_voting.vote(current_tack_vote)

        tack_now = self.tack_voting.tack_now(on_port_tack)

        if tack_now:
            if on_port_tack:
                tack_to = 'tack_to_stbd_tack'
                beating_angle = -self.nav.beating_angle
            else:
                tack_to = 'tack_to_port_tack'
                beating_angle = self.nav.beating_angle
            self.sailing_state = tack_to
            return tack_to, self.nav.wind_angle_to_heading(beating_angle)

        # Continue on our current tack
        goal_wind_angle = wp_wind_angle
        if on_port_tack:
            goal_wind_angle = max(goal_wind_angle, self.nav.beating_angle)
        else:
            # On the starboard tack
            goal_wind_angle = min(goal_wind_angle, -self.nav.beating_angle)

        self.debug_pub('goal_wind_angle', goal_wind_angle)

        return 'normal', self.nav.wind_angle_to_heading(goal_wind_angle)
