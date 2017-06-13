#!/bin/bash
set -e

PIIP="192.168.42.1"
if [ "$1" = "pi3" ]
then
  PIIP="192.168.12.1"
fi

echo "Pushing to the pi at $PIIP..."
git push pi@$PIIP:sailing-robot-bare


echo "Pulling from the pi..."
ssh pi@$PIIP 'cd ~/sailing-robot; git pull bareclone master'
