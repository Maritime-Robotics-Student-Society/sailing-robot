"""Tasks with ROS 2 debugging machinery.

This is separate from the base task running machinery so that that can be tested
without ROS being involved.
"""
import importlib

from .tasks import TasksRunner


class RosTasksRunner(TasksRunner):
    """TasksRunner subclass that integrates with a ROS 2 node for logging and
    debug topic publishing.

    Pass the rclpy Node instance as the ``node`` keyword argument so that
    publishers can be created on it.
    """

    def __init__(self, *args, **kwargs):
        self._node = kwargs.pop('node', None)
        self.debug_topics = {}
        self.register_debug_topics([
            ('task_ix', 'Int16'),
            ('active_task_kind', 'String'),
        ])
        super(RosTasksRunner, self).__init__(*args, **kwargs)

    def log(self, level, msg, *values):
        """Log output through the ROS 2 node logger."""
        if self._node is None:
            print(msg % values)
            return
        logger = self._node.get_logger()
        formatted = msg % values if values else msg
        if level == 'fatal':
            logger.fatal(formatted)
        elif level == 'error':
            logger.error(formatted)
        elif level == 'warning':
            logger.warn(formatted)
        elif level == 'info':
            logger.info(formatted)
        elif level == 'debug':
            logger.debug(formatted)
        else:
            logger.error(formatted)

    def register_debug_topics(self, topics):
        """Set up publishers for a task's debugging topics.

        *topics* should be a list of pairs (topic_name, data_type), e.g.::

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

            if self._node is not None:
                pub = self._node.create_publisher(dt, topic, 10)
            else:
                pub = _NullPublisher()
            self.debug_topics[topic] = (datatype_s, pub)

    def debug_pub(self, topic, value):
        """Publish a value for a debugging topic.

        *topic* should be the name of a topic previously set up by
        :meth:`register_debug_topics`.
        """
        try:
            _datatype, pub = self.debug_topics[topic]
        except KeyError:
            self.log('warning', 'Tried to publish to missing topic: %s', topic)
            return
        pub.publish(value)

    def _make_task(self, taskdict):
        task = super(RosTasksRunner, self)._make_task(taskdict)

        self.register_debug_topics(task.debug_topics)
        task.log = self.log
        task.debug_pub = self.debug_pub
        task.init_ros()
        return task


class _NullPublisher:
    """Stub publisher used when no ROS 2 node is available (e.g. in tests)."""

    def publish(self, _value):
        pass
