import rospy
class Sail_data():
    def __init__(self):
        """

        :rtype: Sail data and update true wind angle functions
        """
        self.sail_sheet_length = 0.
        self.true_wind_anlge = 0.

    def update_true_wind_angle(self, msg):
        self.true_wind_anlge = msg.data
