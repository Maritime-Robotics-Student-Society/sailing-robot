#!/bin/bash

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

python compass_calib


echo '----------------------------------------------------------'
echo "Let's calibrate the wind vane"
echo 'hit enter when ready'
echo '----------------------------------------------------------'
read

python wind_direction_calib


rosparam dump  ../src/sailing_robot/launch/parameters/calibration_${name}.yaml /calibration
