#!/bin/bash


publish_led() {

  rostopic pub -1 /led_blink std_msgs/Int16MultiArray -- "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [$1, $2, $3]" &> /dev/null &

}


rostopic list &> /dev/null

if [ "$?" = 0 ]
then

  echo -e '\033[0;32m' "ROS is running"
  publish_led 0 255  0 # green
else

  echo -e '\033[0;31m' "ROS is not running"
  roscore &> /dev/null &

  rostopic list &> /dev/null
  # wait for roscore to be running and ready
  while [ ! "$?" = 0 ]
  do
    sleep 0.5
    rostopic list &> /dev/null 
  done

  publish_led 255 0  0 # red

#  killall -9 roscore
  killall -9 rosmaster

fi

