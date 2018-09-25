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
import copy
import numpy as np
from scipy.optimize import curve_fit
from scipy import __version__ as scipy_version


class Wave_position():
    def __init__(self, frequency, time_range, refresh_time=1):
        # frequency: rate of acceleration reading
        # time range: readings acquired during this time window are used for predictions
        # refresh time: how often is prediction function re-trained on the most recent data

        if ((int(scipy_version.split('.')[0]) == 0) and (int(scipy_version.split('.')[1]) < 17)):
            raise Exception("Your scipy is outdated. Minimal required version is 0.17.0. Your are currently running "+scipy_version)

        self.period = 1.0/frequency # period in seconds between each two data points
        # queue is updated every time update func is called.
        self.time_range = time_range # time range captured (size of window for fitting the curve)
        self.refresh_time = refresh_time # after this time new fit is performed
        self.queue = deque()
        self.required_queue_length = frequency * time_range
        # xdata & ydata are updated in time_range periods
        self.xdata = np.array([])
        self.ydata = np.array([])
        self.popt = np.array([0, 0, 0])
        self.initializing = True
        self.last_refresh = 0 # time when last refresh occured

    def update(self, data_point):
        # data point, vertical acceleration reading

        self.queue.append(data_point)
        if (self.initializing):
            if (len(self.queue) >= self.required_queue_length):
                self.initializing = False
        else:
            # Delete the oldest value to keep the queue size constant.
            self.queue.popleft()
            now = time.time()
            if ((now - self.last_refresh) >= self.refresh_time):
                try:
                    # Copy current queue to self.ydata.
                    self.process_queue()
                    # Fit model_func to the data.
                    self.train()
                    self.last_refresh = now
                except RuntimeError as e:
                    print(e) 

    def model_func(self, x, a, b, c):
        return a * (np.cos(b * x + c)) + np.mean(self.ydata)

    def train(self):
        """
        Fits model_func to self.xdata & self.ydata and saves fit parameters
        to self.popt.
        """
        guess_freq = 0.5
        guess_amplitude = 3*np.std(self.ydata)/(2**0.5)
        guess_phase = 0
        p0=[guess_amplitude, guess_freq, guess_phase]

        popt, pcov = curve_fit(f=self.model_func, xdata=self.xdata, ydata=self.ydata, p0=p0, bounds=((0, 0, 0), (100., 2., np.pi)), maxfev=2000)
        self.popt = popt

    def process_queue(self):
        """
        Copies current queue to self.ydata a produces corresponding self.xdata
        where difference between two subsequent data points is equal to time period.
        All xdata are negative apart the last value which is 0.
        """
        self.ydata = copy.copy(self.queue)
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
            pi = np.pi
            amplitude, frequency, phase = self.popt
            last_refresh = self.last_refresh
            now = time.time()
            diff = float(now - last_refresh)
            cos_inner = float(frequency * diff + phase)
            rel_distance = cos_inner/pi - int(cos_inner/pi)

            # print("frequency: {}".format(frequency))
            # print("diff: {}".format(diff))
            # print("phase: {}".format(phase))
            # print("cos_inner: {}".format(cos_inner))
            # print("rel_distance: {}".format(rel_distance))
            # print("\n")

            return rel_distance
        else:
            return "initializing"



    ################################################
    ## Debug Methods

    # def generate_data(self):
    #     self.xdata = np.linspace(0, 20, 100)
    #     y = self.model_func(self.xdata, 1, 1, 5, 6)
    #     np.random.seed(1729)
    #     y_noise = 0.2 * np.random.normal(size=self.xdata.size)
    #     self.ydata = y + y_noise
    
    # def plot_all(self):
    #     import matplotlib.pyplot as plt
    #     plt.plot(self.xdata, self.ydata, 'b*', label='training data')
    #     plt.plot(self.xdata, self.model_func(self.xdata, *self.popt), 'r+', label='predicted')
    #     plt.legend(loc=0)
    #     plt.show()

    # def plot_training_data(self):
    #     import matplotlib.pyplot as plt
    #     plt.plot(self.xdata, self.ydata, 'b*')
    #     plt.show()
    
    # def print_vars(self):
    #     print("period: {}\n".format(self.period))
    #     print("time_range: {}\n".format(self.time_range))
    #     print("refresh_time: {}\n".format(self.refresh_time))
    #     print("queue size: {}\n".format(len(self.queue)))
    #     print("xdata size: {}\n".format(len(self.xdata)))
    #     print("ydata size: {}\n".format(len(self.ydata)))
    #     print("queue: {}\n".format(self.queue))
    #     print("xdata: {}\n".format(self.xdata))
    #     print("ydata: {}\n".format(self.ydata))
    #     print("popt: {}\n".format(self.popt))
    #     print("initializing: {}\n".format(self.initializing))
    #     print("last_refresh: {}\n".format(self.last_refresh))

    ################################################
