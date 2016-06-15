#!/bin/sh 

# Install needed packages
sudo apt-get --assume-yes install dphys-swapfile

# Copy default dphys-swapfile
sudo cp /etc/dphys-swapfile /etc/dphys-swapfile.old

# Delete old and create new dphys-swapfile
sudo rm /etc/dphys-swapfile
sudo touch /etc/dphys-swapfile

# Contents of dphys-swapfile
sudo echo "CONF_SWAPSIZE=1024" >> /etc/dphys-swapfile

# Increase swap size
sudo dphys-swapfile setup
sudo dphys-swapfile swapon

# Setup ROS repositories
sudo sh -c \'echo \"deb http://packages.ros.org/ros/ubuntu jessie main\" > /etc/apt/sources.list.d/ros-latest.list\'
wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -

# Update Debian package index
sudo apt-get update
sudo apt-get --assume-yes upgrade

# Install bootstrap dependencies
sudo apt-get --assume-yes install python-pip python-setuptools python-yaml python-distribute python-docutils python-dateutil python-six
yes | sudo pip install rosdep rosinstall_generator wstool rosinstall

# Initialize Rosdep
sudo rosdep init
rosdep update

# Create Catkin Workspace
mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws

# Fetch core packages to be built
rosinstall_generator mavros mavros_extras hector_slam hector_localization hokuyo_node ros_control joystick_drivers ros_comm geometry_msgs sensor_msgs rosserial_python rosserial_msgs diagnostic_msgs --rosdistro indigo --deps --wet-only --exclude roslisp --tar > indigo-ros_comm-wet.rosinstall
wstool init src indigo-ros_comm-wet.rosinstall

# Resolve dependencies with rosdep
cd ~/ros_catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro indigo -y -r --os=debian:jessie

# Build catkin workspace
sudo ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/indigo

# Source new installation
source /opt/ros/indigo/setup.bash
echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc

# Setup networking so other laptops can connect to the boat
# master is the boat
echo "export ROS_HOSTNAME=192.168.42.1" >> ~/.bashrc

cd ~
