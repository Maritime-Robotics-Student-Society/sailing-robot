import unittest

from sailing_robot.tasks import TasksRunner
from sailing_robot.navigation import Navigation
from sailing_robot.heading_planning_laylines import HeadingPlan
from sailing_robot.station_keeping import StationKeeping

tasks_def_1 = [
    {'kind': 'to_waypoint',
     'lat': 50.936981,
     'lon': -1.405315,
    },
    {'kind': 'keep_station',
     'markers': [
        [50.8, 1.01],
        [50.8, 1.03],
        [50.82, 1.01],
        [50.82, 1.03],
     ],
    },
]

tasks_bad = tasks_def_1 + [
    {'kind': 'test_bad'},
]

class TasksTests(unittest.TestCase):
    def test_load(self):
        tr = TasksRunner(tasks_def_1, Navigation())
        self.assertIsInstance(tr.tasks[0], HeadingPlan)
        self.assertIsInstance(tr.tasks[1], StationKeeping)

    def test_load_bad(self):
        with self.assertRaises(ValueError):
            tr = TasksRunner(tasks_bad, Navigation())

    def test_step(self):
        tr = TasksRunner(tasks_def_1, Navigation())
        tr.start_next_task()
        self.assertIsInstance(tr.active_task, HeadingPlan)
        tr.start_next_task()
        self.assertIsInstance(tr.active_task, StationKeeping)
