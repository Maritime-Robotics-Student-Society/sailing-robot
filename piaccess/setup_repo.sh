#!/bin/bash

# Set up the repository clones on a blank raspberry pi.
# Prepare the directories expected by push2pi and pullfrompi.
# This requires an internet connection to clone from Github.

ssh pi@$SAIL_PI_IP <<'ENDSSH'
git clone https://github.com/Maritime-Robotics-Student-Society/sailing-robot.git sailing-robot
git clone --bare sailing-robot sailing-robot-bare
cd sailing-robot
git remote add bareclone ~/sailing-robot-bare
ENDSSH
