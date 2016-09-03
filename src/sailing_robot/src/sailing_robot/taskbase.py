"""Base class for task classes.

This has almost no implementation; the debugging methods are injected by
tasks_ros. This allows us to test task classes outside ROS.
"""

class TaskBase(object):
    debug_topics = []
    
    def debug_pub(self, topic, value):
        pass

    def log(self, level, msg, *values):
        print(msg % values)

    def init_ros(self):
        pass
