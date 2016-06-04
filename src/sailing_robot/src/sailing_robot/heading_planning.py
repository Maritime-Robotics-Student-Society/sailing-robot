import LatLon as ll

from .navigation import Navigation, angleSum, angleAbsDistance

class HeadingPlan:
    def __init__(self, beating_angle=45, tack_line_offset=0.01, utm_zone=30):
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
        self.nav = Navigation(beating_angle=beating_angle, utm_zone=utm_zone)
        self.waypoint = ll.LatLon(50.742810, 1.014469) #somewhere in the solent
        self.tack_line_offset = tack_line_offset
        self.wp_heading = 0
        self.side_heading = 0
        self.alternate_heading = 0
        self.sailing_state = 'normal'  # sailing state can be 'normal','tack_to_port_tack' or  'tack_to_stbd_tack'
    
    def start(self):
        pass

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

        # We're not tacking right now - where do we want to go?
        wp_heading = self.nav.position_ll.heading_initial(self.waypoint)
        wp_wind_angle = self.nav.heading_to_wind_angle(wp_heading)
        if abs(wp_wind_angle) > self.nav.beating_angle:
            # We can sail directly for the next waypoint
            if (wp_wind_angle * boat_wind_angle) > 0:
                # These two have the same sign, so we're on the right tack.
                return ('normal', wp_heading)
            else:
                # We need to tack before going to the waypoint
                if wp_wind_angle > 0:
                    switch_to = 'tack_to_port_tack'
                    beating_angle = self.nav.beating_angle
                else:
                    switch_to = 'tack_to_stbd_tack'
                    beating_angle = -self.nav.beating_angle
                self.sailing_state = switch_to
                return switch_to, self.nav.wind_angle_to_heading(beating_angle)

        # We need to tack upwind to the waypoint.
        if boat_wind_angle > 0:
            # On the port tack
            offset_hdg = angleSum(self.nav.absolute_wind_direction(), 90)
            beating_angle = self.nav.beating_angle
            other_tack = 'tack_to_stbd_tack'
        else:
            offset_hdg = angleSum(self.nav.absolute_wind_direction(), -90)
            beating_angle = -self.nav.beating_angle
            other_tack = 'tack_to_port_tack'
        tack_point = self.waypoint.offset(offset_hdg, self.tack_line_offset)
        boat_from_tack_pt = tack_point.heading_initial(self.nav.position_ll)
        if angleAbsDistance(boat_from_tack_pt, offset_hdg) < 90:
            # Outside the tack corridor. Ready about!
            self.sailing_state = other_tack
            return other_tack, self.nav.wind_angle_to_heading(-beating_angle)
        else:
            # Continue on this tack, sailing as close to the wind as we can.
            return 'normal', self.nav.wind_angle_to_heading(beating_angle)
