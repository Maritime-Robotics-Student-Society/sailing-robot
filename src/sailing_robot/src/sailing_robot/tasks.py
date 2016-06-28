from __future__ import print_function

from LatLon import LatLon
import time

from .navigation import Navigation
from .heading_planning import HeadingPlan
from .station_keeping import StationKeeping

class TimedEnd(object):
    def __init__(self, seconds):
        self.seconds = seconds
        self.ends_at = None
    
    def start(self):
        self.ends_at = time.time() + self.seconds
    
    def check(self):
        return (time.time() > self.ends_at)
    

class TasksRunner(object):
    def __init__(self, tasks, nav, log=print):
        self.task_ix = -1
        self.active_task = None
        self.log = log
        self.nav = nav
        self.tasks = [self._make_task(d) for d in tasks]
    
    def _make_task(self, taskdict):
        kind = taskdict['kind']
        if kind == 'to_waypoint':
            wp = LatLon(taskdict['lat'], taskdict['lon'])
            # Other parameters?
            task = HeadingPlan(waypoint=wp, nav=self.nav)
        elif kind == 'keep_station':
            markers = [tuple(p) for p in taskdict['markers']]
            task = StationKeeping(self.nav, markers,
                            buffer_width=taskdict.get('buffer_width', 10))
        else:
            raise ValueError("Unknown task type: {}".format(kind))
        
        task.task_kind = kind
        return task
    
    def start_next_task(self):
        self.task_ix += 1
        self.active_task = self.tasks[self.task_ix]
        endcond = '' # TODO
        self.log("Running task {}: {} with end condition {}".format(
                    self.task_ix, self.active_task.task_kind, '/'.join(endcond)
        ))
        self.active_task.start()
        
    def calculate_state_and_goal(self):
        if self.active_task.check_end_condition():
            self.start_next_task()
        return self.active_task.calculate_state_and_goal()
