# SSH to the running docker container with X forwarding.
# This works when you have run start-container.sh to start the docker container,
# and allows you to run graphical programs such as rviz. Probably Linux-only.
ssh 172.17.0.2 -l pi -X
