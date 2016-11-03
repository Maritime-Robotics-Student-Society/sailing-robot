#!/bin/sh 

cd ~

# Clone from git
git clone https://github.com/Maritime-Robotics-Student-Society/sailing-robot.git

# Resolve dependencies with rosdep
cd ~/sailing-robot
rosdep install --from-paths src --ignore-src --rosdistro indigo -y -r --os=debian:jessie

# Build catkin workspace
catkin_make

# Source new installation
source devel/setup.bash
echo "source /home/pi/sailing-robot/devel/setup.bash" >> ~/.bashrc

cd ~
