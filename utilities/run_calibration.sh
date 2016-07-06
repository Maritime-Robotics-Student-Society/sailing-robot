#!/bin/bash


#rostopic list 2>&1 /dev/null || echo "roscore needs to be running"; exit 1


defaultname='laser'

echo "Name of the calibration file [" $defaultname "]"
read name

if [ ! "$name" ]
then 
  name=$defaultname
fi


echo '----------------------------------------------------------'
echo 'Calibration of the Compass, be ready to dance!'
echo 'hit enter when ready'
echo '----------------------------------------------------------'

read

python2 calibration_scripts/compasscalib


echo '----------------------------------------------------------'
echo "Let's calibrate the wind vane"
echo 'hit enter when ready'
echo '----------------------------------------------------------'
read

python2 calibration_scripts/wind_direction_calib


rosparam dump  ../src/sailing_robot/launch/parameters/calibration_${name}.yaml /calibration
