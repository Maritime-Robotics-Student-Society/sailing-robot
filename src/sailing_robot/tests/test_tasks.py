import unittest

from sailing_robot.tasks import TasksRunner
from sailing_robot.navigation import Navigation
from sailing_robot.heading_planning_laylines import HeadingPlan
from sailing_robot.station_keeping2 import StationKeeping

tasks_def_1 = [
    {'kind': 'to_waypoint',
     'waypoint_ll': [50.936981, -1.405315]
    },
    {'kind': 'keep_station',
     'marker_ll': [50.8, 1.01],
     'linger': 90,
    },
]

tasks_bad = tasks_def_1 + [
    {'kind': 'test_bad'},
]

class TasksTests(unittest.TestCase):
    def test_load(self):
        tr = TasksRunner(tasks_def_1, Navigation(utm_zone=30))
        self.assertIsInstance(tr.tasks[0], HeadingPlan)
        self.assertIsInstance(tr.tasks[1], StationKeeping)

    def test_load_bad(self):
        with self.assertRaises(ValueError):
            tr = TasksRunner(tasks_bad, Navigation(utm_zone=30))

    def test_step(self):
        tr = TasksRunner(tasks_def_1, Navigation(utm_zone=30))
        tr.start_next_task()
        self.assertIsInstance(tr.active_task, HeadingPlan)
        tr.start_next_task()
        self.assertIsInstance(tr.active_task, StationKeeping)
