## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
## http://docs.ros.org/api/catkin/html/howto/format2/installing_python.html## pdf download of the page in sources folder:
## docs-ros_installing_python.pdf
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['sailing_robot'],
    package_dir={'': 'src'})

setup(**setup_args)

