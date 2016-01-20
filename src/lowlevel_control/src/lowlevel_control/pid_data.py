import rospy


class PID_Data:
    def __init__(self):
        self.goal_heading = 0.
        self.heading = 0.

    def update_goal_heading(self, msg):
        """
        Read in new data for PID controller
        :param msg:
        """
        self.goal_heading = msg.data

    def update_heading(self, msg):
        self.heading = msg.data
