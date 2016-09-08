"""Run a timer while other tasks run"""
from threading import Timer

from .taskbase import TaskBase

class StartTimer(TaskBase):
    def __init__(self, nav, seconds, jump_to, jump_callback):
        """Machinery to stay near a given point.
        
        This is meant to be started when we're already close to the marker; we'll
        normally put it immediately after a to_waypoint task to go to the marker.

        nav : a Navigation object for common machinery.
        marker_ll : a (lat, lon) point marking where we'll try to stay close to.
        linger : time in seconds to stay there
        radius : distance in metres which we'll try to bounce around the marker
        wind_angle : the absolute wind angle to sail (in degrees) when inside
           radius. This will automatically be flipped according to the tack.
        """
        self.nav = nav
        self.seconds = seconds
        self.jump_to = jump_to
        self.jump_callback = jump_callback

    def start(self):
        # Create the timer here so that the task instance can be used more than
        # once.
        self.timer = Timer(seconds, jump_callback, args=[jump_to])
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
