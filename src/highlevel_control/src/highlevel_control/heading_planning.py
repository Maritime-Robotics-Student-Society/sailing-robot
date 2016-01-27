import rospy
import LatLon as ll

class HeadingPlan:
    def __init__(self):
        self.wind_direction = 0.
        self.waypoint = ll.LatLon(50.742810, 1.014469) #somewhere in the solent
        self.position = ll.LatLon(50.8,1.02)
        self.dead_zone = 90 #assuming symmetric dead zone, total angle range of dead zone
        self.heading = 0
        self.wp_heading = self.heading
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
    def calculate_wp_heading(self):
        self.wp_heading = self.position.heading_initial(self.waypoint)

    def calculate_side_headings(self):
        # assume dead zones are +/- 45 degrees upwind
        # (total 'in irons' area: 90 degrees)
        
        headingA = angleSum(self.wind_direction, self.dead_zone/2.)
        headingB = angleSum(self.wind_direction, -self.dead_zone/2.)
        if angleAbsDistance(self.heading, headingA) < angleAbsDistance(self.heading, headingB):
            self.side_heading = headingA
            self.alternate_heading = headingB
        else:
            self.side_heading = headingB
            self.alternate_heading = headingA




    ##################
    #
    # Checks for state determination
    #
    ##################
    def check_outside_dead_zone(self):
        # if the distance to the alternate heading is larger than the dead zone angle
        # the boat must be outside the laylines
        if angleAbsDistance(self.heading, self.alternate_heading) > self.dead_zone:
            return True
        else:
            return False

    def determine_sailing_state(self):
        pass

################
#
# General utility functions
#
################

def angleSum(a,b):
    return (a+b)%360

def angleAbsDistance(a,b):
    distanceA = abs((a - b)%360)
    distanceB = abs((a - b - 360)%360)
    return min(distanceA, distanceB)

