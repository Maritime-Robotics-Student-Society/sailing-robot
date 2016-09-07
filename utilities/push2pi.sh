#!/bin/bash

echo pushing to the pi
git push pi@192.168.42.1:sailing-robot-bare 


echo pulling from the pi
ssh pi@192.168.42.1 'cd ~/sailing-robot; git pull bareclone master'
