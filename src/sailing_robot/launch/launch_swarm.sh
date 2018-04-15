!#/bin/bash

roscore &
#Load swarm parameter file
rosparam load "launch/parameter/swarm"
boatCount=$(rosparam get swarm/boatCount)

for i in $(seq 0 $boatCount)
do
    roslaunch sailing_robot swarm.launch ns:=/boat$i >> /dev/null

done



