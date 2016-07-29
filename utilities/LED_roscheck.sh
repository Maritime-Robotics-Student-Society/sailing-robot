#!/bin/bash


publish_led() {
  shift=300
  r=$1
  g=$2
  b=$3
  intcolor=$(echo "$r*$shift*$shift + $g*$shift + $b" | bc)
  rostopic pub -1 /led_blink std_msgs/Int32 --  $intcolor &> /dev/null &
}

  publish_led 255 0 0 # green

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

  killall -9 roscore
  killall -9 rosmaster

fi

