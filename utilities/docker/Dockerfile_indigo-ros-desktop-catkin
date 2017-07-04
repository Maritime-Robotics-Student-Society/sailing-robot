FROM sotonsailbot/ros:indigo-base-catkin
Maintainer Martin Biggs

USER root
RUN apt-get update \
 && apt-get upgrade --assume-yes \
# TODO narrow down what ROS packages I need
 && apt-get install --assume-yes ros-indigo-desktop 
USER pi
