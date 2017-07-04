docker build -t sotonsailbot/ros:indigo-base-catkin -f ./Dockerfile_indigo-ros-base-catkin ../ && \
docker build -t sotonsailbot/ros:indigo-desktop-catkin -f ./Dockerfile_indigo-ros-desktop-catkin ../ && \
docker build -t sotonsailbot/ros:indigo-ros-tutorials -f ./Dockerfile_indigo-ros-tutorials ../
docker build -t sotonsailbot/ros:indigo -f ./Dockerfile_indigo-sailing-robot ../
