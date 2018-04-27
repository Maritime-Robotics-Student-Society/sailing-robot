"""Code for staying inside a target region"""
import LatLon as ll
from shapely.geometry import Polygon

from .taskbase import TaskBase

class StationKeeping(TaskBase):
    def __init__(self, nav, markers, buffer_width=10):
        """Machinery to stay within a marked area.

        nav is a Navigation object for common machinery.

        markers is a list of (lat, lon) points marking the area we need to stay in.
        
        buffer_width is a distance in metres. The boat will try to stay this
        far inside the boundaries of the target area. This is the margin for
        turning, errors, wind changes, and so on.
        """
        self.nav = nav
        self.markers = markers or [
            (50.8, 1.01),
            (50.8, 1.03),
            (50.82, 1.01),
            (50.82, 1.03),
        ]
        self.target_zone = Polygon([
            self.nav.latlon_to_utm(*p) for p in self.markers
        ])
        self.inner_zone = self.target_zone.buffer(-buffer_width)
        self.goal_heading = 0
        self.sailing_state = 'normal'  # sailing state can be 'normal','switch_to_port_tack' or  'switch_to_stbd_tack'
    
    def start(self):
        pass

    def calculate_state_and_goal(self):
        """Work out what we want the boat to do
        """
        boat_wind_angle = self.nav.angle_to_wind()
        if self.sailing_state != 'normal':
            # A tack is in progress
            if self.sailing_state == 'switch_to_port_tack':
                beating_angle = self.nav.beating_angle
                continue_tack = boat_wind_angle < beating_angle
            else:  # 'switch_to_stbd_tack'
                beating_angle = -self.nav.beating_angle
                continue_tack = boat_wind_angle > beating_angle
            if continue_tack:
                self.goal_heading = self.nav.wind_angle_to_heading(beating_angle)
                return self.sailing_state, self.goal_heading
            else:
                # Tack completed
                self.sailing_state = 'normal'

        if self.nav.position_xy.within(self.inner_zone):
            # We're safe: carry on with our current heading
            return self.sailing_state, self.goal_heading
        
        centroid = self.target_zone.centroid
        centroid_ll = self.nav.utm_to_latlon(centroid.x, centroid.y)
        heading_to_centroid = self.nav.position_ll.heading_initial(centroid_ll)

        centroid_wind_angle = self.nav.heading_to_wind_angle(heading_to_centroid)
        if abs(centroid_wind_angle) > self.nav.beating_angle:
            # We can sail directly towards the centroid
            if (centroid_wind_angle * boat_wind_angle) > 0:
                # These two have the same sign, so we're on the right tack.
                return ('normal', centroid_wind_angle)
            else:
                # We need to tack before going towards the centroid
                if centroid_wind_angle > 0:
                    switch_to = 'switch_to_port_tack'
                    beating_angle = self.nav.beating_angle
                else:
                    switch_to = 'switch_to_stbd_tack'
                    beating_angle = -self.nav.beating_angle
                self.sailing_state = switch_to
                return switch_to, self.nav.wind_angle_to_heading(beating_angle)

        if boat_wind_angle > 0:
            # On the port tack
            beating_angle = self.nav.beating_angle
            other_tack = 'switch_to_stbd_tack'
        else:
            beating_angle = -self.nav.beating_angle
            other_tack = 'switch_to_port_tack'
        if abs(centroid_wind_angle) > 15:
            # Switch to the tack that will take us closest to the centroid
            if (centroid_wind_angle * boat_wind_angle) < 0:
                return other_tack, self.nav.wind_angle_to_heading(-beating_angle)
        
        # Sail as close to the wind as we can on our current tack
        return 'normal', self.wind_angle_to_heading(beating_angle)
