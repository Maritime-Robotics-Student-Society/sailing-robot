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
from .station_keeping2 import StationKeeping
from .return_to_safety import ReturnToSafetyZone
from .obstacle_waypoints import ObstacleWaypoints
from .jibe_tack_now import JibeTackNow
from .timeout import StartTimer

def tasks_from_wps(wp_params):
    target_radius = wp_params['acceptRadius']
    tack_voting_radius = wp_params['tackVotingRadius']
    coordinates = wp_params['table']
    
    def expand_to_waypoint(wpid):
        return {
            'kind': 'to_waypoint',
            'waypoint_ll': coordinates[wpid],
            'target_radius': target_radius,
            'tack_voting_radius': tack_voting_radius,
            'waypoint_id': wpid,
        }

    res = []
    if 'tasks' in wp_params:
        # Long specification: list of tasks
        for wp_task in wp_params['tasks']:
            kind = wp_task['kind']

            if kind == 'to_waypoint':
                lat, lon = coordinates[wp_task['waypoint']]
                expanded_task = expand_to_waypoint(wp_task['waypoint'])
                if 'accept_radius' in wp_task:
                    expanded_task['target_radius'] = wp_task['accept_radius']
                if 'tack_voting_radius' in wp_task:
                    expanded_task['tack_voting_radius'] = wp_task['accept_radius']

            elif kind == 'keep_station':
                lat, lon = coordinates[wp_task['waypoint']]
                expanded_task = {
                    'kind': 'keep_station',
                    'marker_ll': (lat, lon),
                    'linger': wp_task.get('linger', 300),
                    'radius': wp_task.get('radius', 5),
                    'wind_angle': wp_task.get('wind_angle', 75),
                }

            elif kind == 'obstacle_waypoints':
                expanded_task = {
                    'kind': 'obstacle_waypoints',
                    'normal_wp': expand_to_waypoint(wp_task['normal']),
                    'obstacle_wp': expand_to_waypoint(wp_task['obstacle']),
                }
                if 'accept_radius' in wp_task:
                    expanded_task['normal_wp']['target_radius'] = wp_task['accept_radius']
                    expanded_task['obstacle_wp']['target_radius'] = wp_task['accept_radius']
                if 'tack_voting_radius' in wp_task:
                    expanded_task['normal_wp']['tack_voting_radius'] = wp_task['accept_radius']
                    expanded_task['obstacle_wp']['tack_voting_radius'] = wp_task['accept_radius']

            elif kind == 'start_timer':
                expanded_task = wp_task.copy()

            # Copy over any jump label
            expanded_task['jump_label'] = wp_task.get('jump_label', None)
            res.append(expanded_task)

    else:
        # Short specification: just a series of waypoints to go around
        for wpid in wp_params['list']:
            res.append(expand_to_waypoint(wpid))

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
        self._jump_next = None
        self.tasks = [self._make_task(d) for d in tasks]
        self.check_jump_labels()
    
    def check_jump_labels(self):
        jump_labels = set([t.jump_label for t in self.tasks if t.jump_label is not None])
        for t in self.tasks:
            if t.task_kind == 'start_timer':
                if t.jump_to not in jump_labels:
                    raise ValueError('Timer tries to jump to %r, label not found'
                                        % t.jump_to)

    @staticmethod
    def log(level, msg, *values):
        print(msg % values)
    
    def _make_task(self, taskdict):
        """Turn a task dict from params (or from tasks_from_wps) into a task object
        """
        taskdict = taskdict.copy()
        kind = taskdict.pop('kind')
        jump_label = taskdict.pop('jump_label', None)
        if kind == 'to_waypoint':
            wp = LatLon(*taskdict['waypoint_ll'])
            kw = {'target_radius': taskdict.get('target_radius', 2.0),
                  'tack_voting_radius': taskdict.get('tack_voting_radius', 15.),
                  'waypoint_id': taskdict.get('waypoint_id', None),
                 }
            task = HeadingPlan(waypoint=wp, nav=self.nav, **kw)
        elif kind == 'keep_station':
            task = StationKeeping(self.nav, **taskdict)
        elif kind == 'return_to_safety_zone':
            task = ReturnToSafetyZone(self.nav)
        elif kind == 'obstacle_waypoints':
            normal_wp_plan = self._make_task(taskdict['normal_wp'])
            obstacle_wp_plan = self._make_task(taskdict['obstacle_wp'])
            task = ObstacleWaypoints(self.nav, normal_wp_plan, obstacle_wp_plan)
        elif kind == 'start_timer':
            task = StartTimer(self.nav, seconds=taskdict['seconds'],
                      jump_to=taskdict['jump_to'], jump_callback=self.set_jump)
        elif kind == 'jibe_tack_now':
            task = JibeTackNow(nav=self.nav, action=taskdict['action'])
        else:
            raise ValueError("Unknown task type: {}".format(kind))
        
        task.task_kind = kind
        task.jump_label = jump_label
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

    def set_jump(self, label):
        '''Jump callback to jump to task on next time step.'''
        self._jump_next = label
    
    def process_jump(self):
        '''If a jump is set, go to that task, and clear the jump.
        
        Returns True if a jump occurred.
        '''
        if self._jump_next is None:
            return False
        
        label = self._jump_next
        self._jump_next = None
        for i, task in enumerate(self.tasks):
            if task.jump_label == label:
                self.task_ix = i
                self.on_temporary_task = False
                self.active_task = self.tasks[self.task_ix]
                self.active_task.start()
                self.log('warning', "Jumped to task {}: {}".format(
                            self.task_ix, self.active_task.task_kind
                ))
                return True
        
        self.log('error', "Couldn't find jump label %r", label)
        return False

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
        self.log('info', "Running intermediate task: {}".format(taskdict['kind']))

    def calculate_state_and_goal(self):
        """Use the active task to calculate what to do now.
        
        Before using the active task, checks if it should go to the next task.

        If a safety zone is specified, also checks if we're (nearly) out of it.
        """
        self.process_jump()

        if self.active_task.check_end_condition():
            self.start_next_task()

        if self.nav.check_safety_zone() and self.active_task.task_kind != 'return_to_safety_zone':
            # We're about to wander out of the safety zone!
            self.log('warning', 'At edge of safety zone')
            self.insert_task({'kind': 'return_to_safety_zone'})

        self.debug_pub('task_ix', self.task_ix)
        self.debug_pub('active_task_kind', self.active_task.task_kind)

        return self.active_task.calculate_state_and_goal()

    def debug_pub(self, topic, value):
        pass  # Overridden in subclass to send ROS message
