from collections import deque
import time
import numpy as np
from scipy.optimize import curve_fit

class Wave_position():
    def __init__(self, refresh_time=10):
        self.refresh_time = refresh_time
        # queue is updated every time update func is called.
        self.queue = deque()
        # xdata & ydata are updated in refresh_time periods
        self.xdata = np.array([])
        self.ydata = np.array([])
        self.popt = np.array([])
        self.initializing = True
        self.start_time = 0

    def update(self, data_point):
        self.queue.append(data_point)
        if (self.initializing):
            if (len(self.queue) <= 1):
                self.start_time = time.time()
            if ((time.time() - self.start_time) >= self.refresh_time):
                self.initializing = False
        else:
            # Delete the oldest value to keep the queue size constant.
            self.queue.popleft()

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
        self.popt, pcov = curve_fit(self.model_func, self.xdata, self.ydata)

    def plot(self):
        import matplotlib.pyplot as plt
        plt.plot(self.xdata, self.ydata, 'b*', label='data')
        plt.plot(self.xdata, self.model_func(self.xdata, *self.popt), 'r-')
        plt.show()


if __name__ == '__main__':
    wp = Wave_position(5)
    wp.update(1)
    wp.update(2)
    wp.update(3)
    wp.update(4)
    print(wp.queue)
