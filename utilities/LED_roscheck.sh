#!/bin/bash
# This script checks if ROS is currently running
# If ros is running the LEDs on the boat will blink green
# If ros is not running the script will run ros momentanously to blink the LEDs in red
#                 (Note that at the moment I am writing this, I don't think the red 
#         blinking would actually work since the node for blinking is not launched...)
#
# This is done via publishing a /led_blink message, see Naming-conventions in the wiki 
# for more detail on this


publish_led() {
  shift=300
  r=$1
  g=$2
  b=$3
  intcolor=$(echo "$r*$shift*$shift + $g*$shift + $b" | bc)
  rostopic pub -1 /led_blink std_msgs/Int32 --  $intcolor &> /dev/null &
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

  killall -9 roscore
  killall -9 rosmaster

fi

