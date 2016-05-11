import LatLon as ll

class HeadingPlan:
    def __init__(self, beating_angle=45, tack_line_offset=0.01):
        """Heading planning machinery.

        beating_angle is the closest angle we can sail to the wind -
        the total dead zone is twice this angle. Measured in degrees.

        tack_line_offset is half the width of the tacking corridor we use when
        sailing towards a marker upwind, measured in km.
        """
        self.wind_direction = 0.
        self.waypoint = ll.LatLon(50.742810, 1.014469) #somewhere in the solent
        self.position = ll.LatLon(50.8,1.02)
        self.beating_angle = beating_angle
        self.tack_line_offset = tack_line_offset
        self.heading = 0
        self.wp_heading = 0
        self.side_heading = 0
        self.alternate_heading = 0
        self.sailing_state = 'normal'  # sailing state can be 'normal','tack_to_port_tack' or  'tack_to_stbd_tack'

    ###################
    #
    # receive all needed ROS topics
    #
    ###################
    def update_wind_direction(self, msg):
        self.wind_direction = msg.data

    def update_heading(self, msg):
        self.heading = msg.data

    def update_waypoint(self, msg):
        self.waypoint = ll.LatLon(msg.latitude, msg.longitude)

    def update_position(self, msg):
        self.position = ll.LatLon(msg.latitude, msg.longitude)


    ##################
    #
    # calculate various directions
    #
    ##################

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
        res = (heading - self.absolute_wind_direction()) % 360
        if res > 180:
            res -= 360
        return res

    def wind_angle_to_heading(self, wind_angle):
        """Convert angle relative to the wind (+-180) to a compass heading (0-360).
        """
        return angleSum(self.absolute_wind_direction(), wind_angle)

    def calculate_state_and_goal(self):
        """Work out what we want the boat to do
        """
        boat_wind_angle = self.angle_to_wind()
        if self.sailing_state != 'normal':
            # A tack is in progress
            if self.sailing_state == 'tack_to_port_tack':
                beating_angle = self.beating_angle
                continue_tack = boat_wind_angle < beating_angle
            else:  # 'tack_to_stbd_tack'
                beating_angle = -self.beating_angle
                continue_tack = boat_wind_angle > beating_angle
            if continue_tack:
                return self.sailing_state, self.wind_angle_to_heading(beating_angle)
            else:
                # Tack completed
                self.sailing_state = 'normal'

        # We're not tacking right now - where do we want to go?
        wp_heading = self.position.heading_initial(self.waypoint)
        wp_wind_angle = self.heading_to_wind_angle(wp_heading)
        if abs(wp_wind_angle) > self.beating_angle:
            # We can sail directly for the next waypoint
            if (wp_wind_angle * boat_wind_angle) > 0:
                # These two have the same sign, so we're on the right tack.
                return ('normal', wp_heading)
            else:
                # We need to tack before going to the waypoint
                if wp_wind_angle > 0:
                    switch_to = 'tack_to_port_tack'
                    beating_angle = self.beating_angle
                else:
                    switch_to = 'tack_to_stbd_tack'
                    beating_angle = -self.beating_angle
                self.sailing_state = switch_to
                return switch_to, self.wind_angle_to_heading(beating_angle)

        # We need to tack upwind to the waypoint.
        if boat_wind_angle > 0:
            # On the port tack
            offset_hdg = angleSum(self.absolute_wind_direction(), 90)
            beating_angle = self.beating_angle
            other_tack = 'tack_to_stbd_tack'
        else:
            offset_hdg = angleSum(self.absolute_wind_direction(), -90)
            beating_angle = -self.beating_angle
            other_tack = 'tack_to_port_tack'
        tack_point = self.waypoint.offset(offset_hdg, self.tack_line_offset)
        boat_from_tack_pt = tack_point.heading_initial(self.position)
        if angleAbsDistance(boat_from_tack_pt, offset_hdg) < 90:
            # Outside the tack corridor. Ready about!
            self.sailing_state = other_tack
            return other_tack, self.wind_angle_to_heading(-beating_angle)
        else:
            # Continue on this tack, sailing as close to the wind as we can.
            return 'normal', self.wind_angle_to_heading(beating_angle)

################
#
# General utility functions
#
################

def angleSum(a,b):
    return (a+b)%360

def angleAbsDistance(a,b):
    distanceA = abs((a - b) % 360)
    distanceB = abs((b - a) % 360)
    return min(distanceA, distanceB)
