import rospy

class PID_Data:
    def __init__(self):
        self.goal_heading = 0.


    def update_goal_heading(self, msg):
        self.goal_heading = msg.data
