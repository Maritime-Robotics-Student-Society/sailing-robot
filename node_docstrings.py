"""Extract module level docstrings from the node launcher scripts"""
import ast
import os.path

SCRIPTS_DIR = os.path.join('src', 'sailing_robot', 'scripts')

def get_docstring(filename):
    with open(filename) as f:
        m = ast.parse(f.read())
    
    n = m.body[0]
    if isinstance(n, ast.Expr) and isinstance(n.value, ast.Str):
        return n.value.s
    else:
        return ''

for filename in os.listdir(SCRIPTS_DIR):
    ds = get_docstring(os.path.join(SCRIPTS_DIR, filename))
    print(filename)
    print(ds)
