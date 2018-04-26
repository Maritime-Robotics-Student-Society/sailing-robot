#!/bin/sh 

# Setup ROS repositories
sudo sh -c \'echo \"deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main\" > /etc/apt/sources.list.d/ros-latest.list\'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

# Update Debian package index
sudo apt-get update
sudo apt-get --assume-yes upgrade

# Install bootstrap dependencies
sudo apt-get install --assume-yes ros-kinetic-desktop-full
sudo apt-get install --assume-yes python-rosinstall python-rosinstall-generator python-wstool build-essential

#sudo apt-get --assume-yes install python-pip python-setuptools python-yaml python-distribute python-docutils python-dateutil python-six
#yes | sudo pip install rosdep rosinstall_generator wstool rosinstall

# Initialize Rosdep
sudo rosdep init
rosdep update

# Create Catkin Workspace
#mkdir ~/ros_catkin_ws
#cd ~/ros_catkin_ws

# Fetch core packages to be built
#rosinstall_generator mavros mavros_extras hector_slam hector_localization hokuyo_node ros_control joystick_drivers ros_comm geometry_msgs sensor_msgs rosserial_python rosserial_msgs diagnostic_msgs --rosdistro kinetic --deps --wet-only --exclude roslisp --tar > kinetic-ros_comm-wet.rosinstall
#wstool init src kinetic-ros_comm-wet.rosinstall

# Resolve dependencies with rosdep
#cd ~/ros_catkin_ws
#rosdep install --from-paths src --ignore-src --rosdistro kinetic -y -r --os=debian:jessie

# Build catkin workspace
#sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic

# Source new installation
source /opt/ros/kinetic/setup.bash
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

# Setup networking so other laptops can connect to the boat
# master is the boat
echo "export ROS_HOSTNAME=192.168.42.1" >> ~/.bashrc

