import rospy
#import LatLon

class HeadingPlan:
    def __init__(self):
        self.wind_direction = 0.
        self.waypoint = {'lat':50.742810 , 'lon':-1.014469} #somewhere in the solent
        self.position = {'lat':50.8, 'lon':-1.02}
        self.heading = 0
        self.wp_heading = self.heading
        self.port_layline = 0
        self.stb_layline = 0
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
        self.waypoint['lat'] = msg.latitude
        self.waypoint['long'] = msg.longitude

    def update_position(self, msg):
        self.position['lat'] = msg.latitude
        self.position['long'] = msg.longitude


    ##################
    #
    # calculate various directions
    #
    ##################
    def calculate_laylines(self):
        # assume laylines are +/- 45 degrees upwind
        # (total 'in irons' area: 90 degrees)
        self.port_layline = (self.wind_direction - (180 - 45)) % 360
        self.stbd_layline = (self.wind_direction - (180 + 45)) % 360

    def calculate_wp_heading(self):
        pass

    def calculate_side_heading(self):
        pass

    def calculate_alternate_heading(self):
        pass

    ##################
    #
    # Checks for state determination
    #
    ##################
    def check_outside_laylines(self):
        return True

    def determine_sailing_state(self):
        pass


