# Start the ROS docker container, with this repository mounted inside it as
# $HOME/sailing-robot

# This script lives in utilities/docker/ 
# The sailing-robot repo is two levels up.
this_file=$(readlink -f "$0")
docker=$(dirname "$this_file")
utils=$(dirname "$docker")
repo=$(dirname "$utils")

echo "Starting docker container with $repo mounted at ~/sailing-robot"

docker run --name sailing-robot -h sailing-robot -v "$repo:/home/pi/sailing-robot"\
      -p 8448:8448 --rm -it sotonsailbot/ros:indigo
