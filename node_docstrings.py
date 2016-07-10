"""Extract module level docstrings from the node launcher scripts"""
import ast
import os.path

SCRIPTS_DIR = os.path.join('src', 'sailing_robot', 'scripts')

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

