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
                self.ydata = self.queue.copy()
                l = len(self.ydata)
                self.xdata = np.linspace((-l + 1) * self.period, 0, l)
                # Fit model_func to the data.
                self.popt, pcov = curve_fit(self.model_func, self.xdata, self.ydata)

    def model_func(self, x, a, b):
        return a * np.sin(b * x)



    ################################################
    ## Debug methods
    ################################################

    def produce_data(self):
        self.xdata = np.linspace(0, 20, 100)
        y = self.model_func(self.xdata, 1, 1)
        np.random.seed(1729)
        y_noise = 0.2 * np.random.normal(size=self.xdata.size)
        self.ydata = y + y_noise

    def fit(self):
        self.popt, pcov = curve_fit(self.model_func, self.xdata, self.ydata)

    def plot(self):
        import matplotlib.pyplot as plt
        plt.plot(self.xdata, self.ydata, 'b*', label='data')
        plt.plot(self.xdata, self.model_func(self.xdata, *self.popt), 'r-')
        plt.show()


if __name__ == '__main__':
    wp = Wave_position(frequency=10, time_range=5, refresh_time=1)
    wp.update(-3)
    wp.update(-2)
    wp.update(-1)
    wp.update(0)
    wp.update(1.5)
    wp.update(2.1)
    wp.update(3.3)
    wp.update(2.7)
    print(wp.queue)
