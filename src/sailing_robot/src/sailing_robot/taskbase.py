class TaskBase(object):
    debug_topics = []
    
    def debug_pub(self, topic, value):
        pass

    def log(self, level, msg, *values):
        print(msg % values)
