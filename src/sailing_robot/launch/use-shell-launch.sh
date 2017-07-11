#!/bin/bash
# set prefix to be used for all collected information
fileprefix="foobar"

# paths on the pi
# for simulation on your machine: replace these with your local folders
# define where logs should go
logpath="/home/pi/logs/"
# define where the parameters are loaded from
parampath="/home/pi/sailing-robot/src/sailing-robot/launch/parameters/"
# save the start time for timestamping all files
timestamp="_$(date +%Y-%m-%d_%H:%M:%S)"

# set some colors
GREEN='\033[0;32m'
NC='\033[0m'

# set ros log directory to timestamped directory
export ROS_LOG_DIR=$logpath$fileprefix$timestamp

# start roscore, move to background, and give it a moment to start
roscore &
roscoreID=$!
sleep 2
echo ${GREEN} started roscore ${NC}

# load parameters
rosparam load ${parampath}default.yaml
rosparam load ${parampath}calibration_blackpython.yaml /calibration
# below this, load any parameters that are specific for this mission

# record rosbag with the timestamp we got
rosbag record -a -O ${logpath}${fileprefix}${timestamp}.bag &
echo ${GREEN} rosbag recording started ${NC}

#
# insert further separate ros node runs here
# move every node in the background by adding '&' at the end of line
# save away their process ID and remember to kill in the end, before killing rosnode
# (exception: rosbag - it is hard to kill and needs special treatment)

# launch the main launch file
# this should contain all nodes that have required="true", 
# since it controls the timing of the rest of the script

#
# after finishing the mission, before killling roscore:
#

# saving parameters at the end helps catching rogue parameter settings
rosparam dump ${logpath}${fileprefix}${timestamp}_PARAM

# rosbag is hard to kill, so let's get rid of that one first
# (SIGINT usually doesn't work, and KILL will leave .bag.active instead of .bag)
rosbagnode=$(rosnode list | grep record)
rosnode kill $rosbagnode

# stop roscore
kill -INT $roscoreID

# now all is stopped, it may be worth saving away this file, too
scriptname=`basename "$0"`
cp ./$basename ${logpath}${fileprefix}${timestamp}_sh
