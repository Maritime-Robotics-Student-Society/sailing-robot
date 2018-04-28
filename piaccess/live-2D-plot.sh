#!/bin/bash
set -e

echo "Connecting to ROS at ${SAIL_PI_IP:=192.168.12.1}, launching 2D plot node locally..."
export ROS_IP=$SAIL_PI_IP
export ROS_MASTER_URI=http://$SAIL_PI_IP:11311

rosrun sailing_robot debugging_2D_plot_matplot 
