"""
Wave_position class helps to time a tack manoeuvre. It inputs vertical acceleration and predicts ship
position on the wave so that tack manoevre can be started at the crest when it is most convenient.

Usage:
wp = Wave_position(frequency, time_range, refresh_time) # to initialize
# Every time new reading is obtain, call
wp.update(new_reading) # where new_reading is vertical acceleration
# Call
wp.get_position()
# to get predicted position on the wave. Returns a number between 0 and 1 which is a relative distance
# from the last crest. (at the crest it is 0, at the trough 0.5, just before the next crest 1 and just after
# the crest 0 again). Be aware that during first time_range seconds after first call of update method,
# initialisation is enabled and get_positon() returns "initializing"

Code explanation:
After get_positon is called for the first time, initialisation phase is enabled for the time specified by time_range.
Every time update method is called, new reading is appended to a queue. If get_position is called now, it returns
"initializing". After time_range seconds pass, initializing phase is disabled. Every time update method is called
now, it appends new element to the queue and removes the oldest one to keep the size of the queue constant. Data from
the queue are coppied to ydata variable every time update is called and time from the last update is larger
then refresh_time. When this happens, xdata are generated based on the frequency and number of items in ydata.
Next sine function is fitted to xdata, ydata. Prediction of the position on the wave is made based on this sine
function and time that has passed from the last refresh.
"""

from collections import deque
import time
import numpy as np
from scipy.optimize import curve_fit

class Wave_position():
    def __init__(self, frequency, time_range, refresh_time=1):
        # frequency of acceleration reading
        # time range, readings aquired during this time window are used for predictions (idealy should be a few sea waves)
        # refresh time, how often is prediction function re-trained on the most recent data

        self.period = 1.0/frequency # period in seconds between each two data points
        # queue is updated every time update func is called.
        self.time_range = time_range # time range captured (size of window for fitting the curve)
        self.refresh_time = refresh_time # after this time new fit is performed
        self.queue = deque()
        # xdata & ydata are updated in time_range periods
        self.xdata = np.array([])
        self.ydata = np.array([])
        self.popt = np.array([])
        self.initializing = True
        self.last_refresh = 0 # time when last refresh occured

    def update(self, data_point):
        # data point, vertical acceleration reading
        
        self.queue.append(data_point)
        if (self.initializing):
            now = time.time()
            if (len(self.queue) <= 1):
                self.last_refresh = now
            if ((now - self.last_refresh) >= self.time_range):
                self.initializing = False
        else:
            # Delete the oldest value to keep the queue size constant.
            self.queue.popleft()
            now = time.time()
            if ((now - self.last_refresh) >= self.refresh_time):
                self.last_refresh = now
                # Copy current queue to self.ydata.
                self.process_queue()
                # Fit model_func to the data.
                self.train()


    def model_func(self, x, a, b, c, d):
        return a * (np.cos(b * x + c)) + d

    def train(self):
        """
        Fits model_func to self.xdata & self.ydata and saves fit parameters
        to self.popt.
        """
        self.popt, pcov = curve_fit(self.model_func, self.xdata, self.ydata)

    def process_queue(self):
        """
        Copies current queue to self.ydata a produces corresponding self.xdata
        where difference between two subsequent data points is equal to time period.
        All xdata are negative apart the last value which is 0.
        """
        self.ydata = self.queue.copy()
        l = len(self.ydata)
        self.xdata = np.linspace((-l + 1) * self.period, 0, l)

    def get_position(self):
        """
        Returns predicted position on the wave (number 0-1).
        Returns 0 for position on the crest of wave. While the ship is going down
        to the trough, this number is increasing. After reaching the trough and
        going to another crest this number is still continously incresing until
        it is 1 at the crest, where it is set back to 0.
        (It is relative distance from the last crest.)
        """
        if (not self.initializing):
            a, b, c, d = self.popt
            pi = np.pi
            now = time.time()
            diff = float(now - self.refresh_time)
            cos_inner = float(b * diff + c)
            rel_distance = cos_inner/pi - int(cos_inner/pi)
            return rel_distance
        else:
            return "initializing"



    ################################################
    ## Debug methods
    ################################################

    def generate_data(self):
        self.xdata = np.linspace(0, 20, 100)
        y = self.model_func(self.xdata, 1, 1, 5, 6)
        np.random.seed(1729)
        y_noise = 0.2 * np.random.normal(size=self.xdata.size)
        self.ydata = y + y_noise

    def plot(self):
        import matplotlib.pyplot as plt
        plt.plot(self.xdata, self.ydata, 'b*', label='data')
        plt.plot(self.xdata, self.model_func(self.xdata, *self.popt), 'r+')
        plt.show()

    def print_vars(self):
        print("period: {}\n".format(self.period))
        print("time_range: {}\n".format(self.time_range))
        print("refresh_time: {}\n".format(self.refresh_time))
        print("queue: {}\n".format(self.queue))
        print("xdata: {}\n".format(self.xdata))
        print("ydata: {}\n".format(self.ydata))
        print("popt: {}\n".format(self.popt))
        print("initializing: {}\n".format(self.initializing))
        print("last_refresh: {}\n".format(self.last_refresh))



# TODO: - model_func fits well data that are one period of sine function, however,
#       if several periods are passed, the fit is faulty.
#       See the test cases below.
#
#       - get_position() needs to be tested



if __name__ == '__main__':
    wp = Wave_position(frequency=10, time_range=10, refresh_time=1)

    ########################################################
    ## this works well
    wp.queue = deque(1*[0,3,6,3,0])
    ########################################################

    ########################################################
    ## this doesn't work
    # wp.queue = deque(2*[0,3,6,3,0])
    ########################################################

    ########################################################
    ## this doesn't work either
    # wp.queue = deque([5, 4, 2.7, 1, 0.5, 1.2, 3.2, 4.5, 5, 3.4, 2.5, 1.5, 1, 1.7, 3, 3.7, 4.1])
    ########################################################


    ########################################################
    ## this works well
    # o = 5
    # wp.queue = deque([-2 + o, -1 + o, 0 + o, 1 + o, 2 + o, 1 + o, 0 + o, -1 + o])
    ########################################################

    ########################################################
    ## this works well either
    ## ! line below with wp.process_queue() has to be commented
    # wp.generate_data()
    ########################################################

    wp.process_queue()
    wp.train()
    wp.plot()
