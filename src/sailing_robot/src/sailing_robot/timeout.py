"""Run a timer while other tasks run"""
from threading import Timer

from .taskbase import TaskBase

class StartTimer(TaskBase):
    def __init__(self, nav, seconds, jump_to, jump_callback):
        """Machinery to jump to a waypoint after a set amount of time.

        nav : a Navigation object for common machinery.
        seconds : time in seconds to jump after task is started
        jump_to : Waypoint ID to jump to.
        jump_callback : Function to do jump. Provided in tasks.py
        """
        self.nav = nav
        self.seconds = seconds
        self.jump_to = jump_to
        self.jump_callback = jump_callback

    def start(self):
        # Create the timer here so that the task instance can be used more than
        # once.
        self.timer = Timer(self.seconds, self.jump_callback, args=[self.jump_to])
        # If the tasks process dies, the timer thread should die
        self.timer.daemon = True
        self.timer.start()

    def check_end_condition(self):
        "Are we done yet?"
        return True # Go straight to next task

    def calculate_state_and_goal(self):
        """Work out what we want the boat to do
        """
        # Dummy, should only be used for 0.1s, as it goes immediately to next
        # task
        return 'normal', self.nav.heading
