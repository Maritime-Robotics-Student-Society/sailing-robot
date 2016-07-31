"""Machinery to step through tasks.

This can be tested without ROS; tasks_ros.py contains a subclass which
integrates with ROS to publish logging and debugging topics.
"""

from __future__ import print_function

from LatLon import LatLon
import time
import types

from .navigation import Navigation
from .heading_planning_laylines import HeadingPlan
from .station_keeping import StationKeeping

def tasks_from_wps(wp_params):
    target_radius = wp_params['acceptRadius']
    tack_voting_radius = wp_params['tackVotingRadius']
    coordinates = wp_params['table']

    res = []
    for wpid in wp_params['list']:
        lat, lon = coordinates[wpid]
        res.append({
            'kind': 'to_waypoint',
            'lat': lat,
            'lon': lon,
            'target_radius': target_radius,
            'tack_voting_radius': tack_voting_radius
        })
    return res

class TimedEnd(object):
    def __init__(self, seconds):
        self.seconds = seconds
        self.ends_at = None
    
    def start(self):
        self.ends_at = time.time() + self.seconds
    
    def check(self):
        return (time.time() > self.ends_at)

class TasksRunner(object):
    def __init__(self, tasks, nav):
        self.task_ix = -1
        self.active_task = None
        self.nav = nav
        self.tasks = [self._make_task(d) for d in tasks]

    @staticmethod
    def log(level, msg, *values):
        print(msg % values)
    
    def _make_task(self, taskdict):
        """Turn a task dict from params (or from tasks_from_wps) into a task object
        """
        kind = taskdict['kind']
        if kind == 'to_waypoint':
            wp = LatLon(taskdict['lat'], taskdict['lon'])
            kw = {'target_radius': taskdict.get('target_radius', 2.0), 'tack_voting_radius': taskdict.get('tack_voting_radius', 15.)}
            task = HeadingPlan(waypoint=wp, nav=self.nav, **kw)
        elif kind == 'keep_station':
            markers = [tuple(p) for p in taskdict['markers']]
            task = StationKeeping(self.nav, markers,
                            buffer_width=taskdict.get('buffer_width', 10))
        else:
            raise ValueError("Unknown task type: {}".format(kind))
        
        task.task_kind = kind
        return task
    
    on_temporary_task = False

    def start_next_task(self):
        """Step to the next task, making it the active task.
        """
        self.task_ix += 1
        if self.task_ix >= len(self.tasks):
            self.log('warning', "Run all tasks, returning to start")
            self.task_ix = 0

        self.active_task = self.tasks[self.task_ix]
        self.on_temporary_task = False
        endcond = '' # TODO
        self.log('info', "Running task {}: {} with end condition {}".format(
                    self.task_ix, self.active_task.task_kind, '/'.join(endcond)
        ))
        self.active_task.start()

    def insert_task(self, taskdict):
        """Do a temporary task.

        After completing the temporary task, control will be return to the
        regular task that was active before the temporary task was started.
        """
        # Decrease task_ix so we go back to the current task when this is done.
        if not self.on_temporary_task:
            self.task_ix -= 1
        self.on_temporary_task = True
        self.active_task = self._make_task(taskdict)
        self.active_task.start()
        self.log('info', "Running intermediate task: {}".format(task.task_kind))

    def calculate_state_and_goal(self):
        """Use the active task to calculate what to do now.
        
        Before using the active task, checks if it should go to the next task.
        """
        if self.active_task.check_end_condition():
            self.start_next_task()
        return self.active_task.calculate_state_and_goal()
