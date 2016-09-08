"""Tasks with ROS debugging machinery.

This is separate from the base task running machinery so that that can be tested
without ROS being involved.
"""
import importlib
import rospy

from .tasks import TasksRunner

class RosTasksRunner(TasksRunner):
    def __init__(self, *args, **kwargs):
        self.debug_topics = {}
        self.register_debug_topics([
            ('task_ix', 'Int16'),
            ('active_task_kind', 'String'),
        ])
        super(RosTasksRunner, self).__init__(*args, **kwargs)

    @staticmethod
    def log(level, msg, *values):
        """Log output through ROS."""
        if level == 'fatal':
            rospy.logfatal(msg, *values)
        elif level == 'error':
            rospy.logerr(msg, *values)
        elif level == 'warning':
            rospy.logwarn(msg, *values)
        elif level == 'info':
            rospy.loginfo(msg, *values)
        elif level == 'debug':
            rospy.logdebug(msg, *values)
        else:
            rospy.logerr(msg, *values)

    def register_debug_topics(self, topics):
        """Sets up publishers for a task's debugging topics.
        
        *topics* should be a list of pairs, (topic_name, data_type), e.g.:
        
            [('next_wp', 'sensor_msgs.msg:NavSatFix')]
        """
        for (topic, datatype_s) in topics:
            if (topic in self.debug_topics) \
                    and (self.debug_topics[topic][0] == datatype_s):
                continue  # Already registered
            
            if ':' in datatype_s:
                dt_mod, dt_cls = datatype_s.split(':', 1)
            else:
                dt_mod = 'std_msgs.msg'
                dt_cls = datatype_s
            mod = importlib.import_module(dt_mod)
            dt = getattr(mod, dt_cls)
            
            pub = rospy.Publisher(topic, dt, queue_size=10)
            self.debug_topics[topic] = (datatype_s, pub)

    def debug_pub(self, topic, value):
        """Publish a value for a debugging topic.
        
        *topic* should be the name of a topic previously set up by
        register_debug_topics()
        """
        try:
            datatype, pub = self.debug_topics[topic]
        except KeyError:
            self.log('warning', 'Tried to publish to missing topic: %s', topic)

        pub.publish(value)

    def _make_task(self, taskdict):
        task = super(RosTasksRunner, self)._make_task(taskdict)
        
        self.register_debug_topics(task.debug_topics)
        task.log = self.log
        task.debug_pub = self.debug_pub
        task.init_ros()
        return task
