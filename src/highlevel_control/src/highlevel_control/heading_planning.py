import rospy

class SailToWP:
    def __init__(self):
        self.wind_direction = 0.
        self.next_wp = {'lat':0 , 'lon':0}
        self.heading = 0


    def update_wind_direction(self, msg):
        self.goal_heading = msg.data

   def update_heading(self, msg):
        self.heading = msg.data


 
