#!/bin/bash
set -e

echo "On ${SAIL_PI_IP:=192.168.12.1}, pushing to bareclone..."
ssh pi@$SAIL_PI_IP 'cd ~/sailing-robot; git push bareclone master'

echo "Pulling from the pi..."
git pull pi@$SAIL_PI_IP:sailing-robot-bare master
