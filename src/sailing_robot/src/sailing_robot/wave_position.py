from collections import deque
import time
import numpy as np
from scipy.optimize import curve_fit

class Wave_position():
    def __init__(self, frequency, time_range, refresh_time=1):
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



# TODO: model_func fits well data that are one period of sine function, however,
#       if several periods are passed, the fit is faulty.



if __name__ == '__main__':
    wp = Wave_position(frequency=10, time_range=5, refresh_time=1)
    # wp.queue = deque([6,3,0,3,6,3,0,3,6])
    # wp.queue = deque([5, 4, 2.7, 1, 0.5, 1.2, 3.2, 4.5, 5, 3.4, 2.5, 1.5, 1, 1.7, 3, 3.7, 4.1])
    # wp.queue = deque(1*[0,3,6,3,0])
    # wp.queue = deque(4*[-1, -0.5, 0, 0.5, 1, 0.5, 0, -0.5])

    o = 5
    wp.queue = deque([-2 + o, -1 + o, 0 + o, 1 + o, 2 + o, 1 + o, 0 + o, -1 + o])

    wp.process_queue()
    wp.train()
    wp.plot()
