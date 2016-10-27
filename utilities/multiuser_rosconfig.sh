#!/bin/bash
# This script gives a unique project folder per user based on its
# ip or mac adress
#
# add: source /path/to/this/script
# at the end of the .bashrc
# LICENSE MIT


# use either the ip or the mac adress to distinguish users
USE_IP=1


ip=$(awk '{print $1}' <<< $SSH_CLIENT)


if [ -z "$SSH_CLIENT"  ] || [ "$ip" = "127.0.0.1" ]
then

  port=11311

else

  if [ "$USE_IP" = 1 ]
  then
    ip_hash=$(sed 's/\.//g' <<< $ip)
    port=$((11312 + (ip_hash % 2000)))
  else
    mac_hash=$(arp -a | grep "$ip" | awk '{print $4}' | sed 's/://g;s/[a-z]/1/g;s/^0*//g')
    port=$((11312 + (mac_hash % 2000)))
  fi

fi

echo ROS port: $port
export ROS_MASTER_URI=http://127.0.0.1:$port/
folder="$HOME/sailing-robot_$port"

if [ -d "$folder/sailing-robot" ]
then

  cd $folder/sailing-robot
  echo "Your personal folder is $folder"
  source $folder/sailing-robot/devel/setup.bash

else

  mkdir -p "$folder"
  cd "$folder"
  git clone https://github.com/Maritime-Robotics-Student-Society/sailing-robot.git
  cd sailing-robot
  catkin_make
  source devel/setup.bash
  echo "  This is your first log in, welcome!
  your personal folder is $folder
  have fun!"

fi

