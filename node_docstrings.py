"""Extract module level docstrings from the node launcher scripts"""
import ast
import os.path

SCRIPTS_DIR = os.path.join('src', 'sailing_robot', 'scripts')

topics = {}

class topic(object):
  	def __init__(self):
    	self.publishers = []
        self.subscribers = []
        
#    def addSubscriber(scriptName):
#      	self.subscribers.append(scriptName)
#    def addPublisher(scriptName):
#      	self.publishers.append(scriptName)

def get_docstring(filename):
    with open(filename) as f:
        print(filename)
        try:
            m = ast.parse(f.read())
        except:
            pass
    
    n = m.body[0]
    if isinstance(n, ast.Expr) and isinstance(n.value, ast.Str):
        return n.value.s
    else:
        return ''
      

def get_subscriber(filename):
    with open(filename) as f:
        try:
            data = ast.parse(f.read())
        except:
            pass
      
    return parse_tree(data, 'Subscriber')
      
 
def get_publisher(filename):
    with open(filename) as f:
        try:
            data = ast.parse(f.read())
        except:
            pass
      
    return parse_tree(data, 'Publisher')
      
         

def parse_tree(tree, func_name): #func_name being 'Subscriber' or 'Publisher'
    topic_list = []
    for elem in tree.body:
        if (isinstance(elem, ast.Expr) or isinstance(elem, ast.Assign)) and \
                isinstance(elem.value, ast.Call) and \
                'attr' in dir(elem.value.func) and \
                'id' in dir(elem.value.func.value) and \
                elem.value.func.value.id == 'rospy' and \
                elem.value.func.attr == func_name:

            topic_name = elem.value.args[0].s
            topic_type = str(elem.value.args[1].id)
            topic_list.append({'name': topic_name, 'type': topic_type})
            continue

        if 'body' in dir(elem):
            topic_list =  parse_tree(elem, func_name) + topic_list
          
    return topic_list


      

def process_scripts(SCRIPTS_DIR, topics):
    for nodename in os.listdir(SCRIPTS_DIR):
        newSubscribers = get_subscriber(SCRIPTS_DIR + nodename)
        newPublishers = get_publisher(SCRIPTS_DIR + nodename)
        for topicName in newSubscribers:
            if topicName not in topics:
                topics[topicName] = topic()
            topics[topicName].subscribers.append(topicName)
        for topicName in newPublishers:
            if topicName not in topics:
                topics[topicName] = topic()
            topics[topicName].publishers.append(topicName)        
                
    return topics
        

     
      
  
# write header of node overview file

nodeOverview = open('src/sailing_robot/doc/nodes.rst', 'w')
nodeOverview.write("Available ROS Nodes\n")
nodeOverview.write("===================\n")
nodeOverview.write("\n")
nodeOverview.write(".. toctree::\n")
nodeOverview.write("   :maxdepth: 2\n")
nodeOverview.write("\n")
nodeOverview.close()
                      
for nodename in os.listdir(SCRIPTS_DIR):
    ds = get_docstring(os.path.join(SCRIPTS_DIR, nodename))
    nodeOverview = open('src/sailing_robot/doc/nodes.rst', 'a')
    nodeOverview.write("   " + nodename + '\n')
    nodeOverview.close()


    nodeFile = open('src/sailing_robot/doc/' + nodename +'.rst', 'w')
    nodeFile.write(nodename + '\n')
    underline = len(nodename) * '='
    nodeFile.write(underline + '\n')
    nodeFile.write(ds)
    nodeFile.close()
                      
nodeOverview = open('src/sailing_robot/doc/nodes.rst', 'w')                      
nodeOverview.write("All currently used ROS Topics\n")
nodeOverview.write("=============================\n")
nodeOverview.write("\n")
nodeOverview.write(".. toctree::\n")
nodeOverview.write("   :maxdepth: 2\n")
nodeOverview.write("\n")

                      
nodeOverview = open('src/sailing_robot/doc/nodes.rst', 'w')                      
for topicName in topics:
    nodeOverview.write(topicName)
    
nodeOverview.close()

