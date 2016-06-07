"""Code for staying inside a target region"""
import LatLon as ll
import shapely
import pyproj

class HeadingPlan:
    def __init__(self, beating_angle=45, markers=(), utm_zone=30,
                buffer_width=10):
        """Heading planning machinery.

        beating_angle is the closest angle we can sail to the wind -
        the total dead zone is twice this angle. Measured in degrees.

        markers is a list of (lat, lon) points marking the area we need to stay in.
        
        utm_zone is the zone number of the UTM system to use. Southampton is in
        zone 30, Portugal in zone 29. http://www.dmap.co.uk/utmworld.htm
        Distance calculations will be less accurate the further from the
        specified zone you are.
        
        buffer_width is a distance in metres. The boat will try to stay this
        far inside the boundaries of the target area. This is the margin for
        turning, errors, wind changes, and so on.
        """
        self.projection = pyproj.Proj(proj='utm', zone=utm_zone, ellps='WGS84')
        self.wind_direction = 0.
        self.markers = markers or [
            (50.8, 1.01),
            (50.8, 1.03),
            (50.82, 1.01),
            (50.82, 1.03),
        ]
        self.target_zone = shapely.Polygon([
            self.latlon_to_utm(*p) for p in self.markers
        ])
        self.inner_zone = self.target_zone.buffer(-buffer_width)
        self.position_ll = ll.LatLon(50.8, 1.02)
        self.position_xy = shapely.Point(self.latlon_to_utm(50.8, 1.02))
        self.beating_angle = beating_angle
        self.heading = 0
        self.goal_heading = 0
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
        self.position_ll = ll.LatLon(msg.latitude, msg.longitude)
        x, y = self.latlon_to_utm(msg.latitude, msg.longitude)
        self.position_xy = shapely.Point(x, y)

    def latlon_to_utm(self, lat, lon):
        """Returns (x, y) coordinates in metres"""
        return self.projection(lon.decimal_degree, lat.decimal_degree)
    
    def utm_to_latlon(self, x, y):
        """Returns a LatLon object"""
        lon, lat = self.projection.inverse(x, y, inverse=True)
        return ll.LatLon(lat, lon)

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
                self.goal_heading = self.wind_angle_to_heading(beating_angle)
                return self.sailing_state, self.goal_heading
            else:
                # Tack completed
                self.sailing_state = 'normal'

        if self.position_xy.within(self.inner_zone):
            # We're safe: carry on with our current heading
            return self.sailing_state, self.goal_heading
        
        centroid = self.target_zone.centroid
        centroid_ll = self.utm_to_latlon(centroid.x, centroid.y)
        heading_to_centroid = self.position_ll.heading_initial(centroid_ll)

        centroid_wind_angle = self.heading_to_wind_angle(heading_to_centroid)
        if abs(centroid_wind_angle) > self.beating_angle:
            # We can sail directly towards the centroid
            if (centroid_wind_angle * boat_wind_angle) > 0:
                # These two have the same sign, so we're on the right tack.
                return ('normal', centroid_wind_angle)
            else:
                # We need to tack before going towards the centroid
                if centroid_wind_angle > 0:
                    switch_to = 'tack_to_port_tack'
                    beating_angle = self.beating_angle
                else:
                    switch_to = 'tack_to_stbd_tack'
                    beating_angle = -self.beating_angle
                self.sailing_state = switch_to
                return switch_to, self.wind_angle_to_heading(beating_angle)

        if boat_wind_angle > 0
            # On the port tack
            beating_angle = self.beating_angle
            other_tack = 'tack_to_stbd_tack'
        else:
            beating_angle = -self.beating_angle
            other_tack = 'tack_to_port_tack'
        if abs(centroid_wind_angle) > 15:
            # Switch to the tack that will take us closest to the centroid
            if (centroid_wind_angle * boat_wind_angle) < 0:
                return other_tack, self.wind_angle_to_heading
        
        # Sail as close to the wind as we can on our current tack
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
