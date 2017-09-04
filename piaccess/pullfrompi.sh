#!/bin/bash
set -e

echo "On ${SAIL_PI_IP:=192.168.12.1}, pushing to bareclone..."
git pull pi@$SAIL_PI_IP:sailing-robot-bare master


echo "Pulling from the pi..."
ssh pi@$SAIL_PI_IP 'cd ~/sailing-robot; git pull bareclone master'
