from collections import deque
import time

class Wave_position():
    def __init__(self, refresh_time=10):
        self.refresh_time = refresh_time
        self.live_queue = deque()
        self.initializing = True
        self.start_time = 0

    def update(self, data_point):
        self.live_queue.append(data_point)
        if (self.initializing):
            if (len(self.live_queue) <= 1):
                self.start_time = time.time()
            if ((time.time() - self.start_time) >= self.refresh_time):
                self.initializing = False
        else:
            # Delete the oldest value to keep the queue size constant.
            self.live_queue.popleft()



if __name__ == '__main__':
    wp = Wave_position(5)
    wp.update(1)
    wp.update(2)
    wp.update(3)
    wp.update(4)
    print(wp.live_queue)
