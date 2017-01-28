docker build -t jamak9/sailing-robot:indigo-base-catkin -f ./Dockerfile_indigo-ros-base-catkin ../ && \
docker build -t jamak9/sailing-robot:indigo-desktop-catkin -f ./Dockerfile_indigo-ros-desktop-catkin ../ && \
docker build -t jamak9/sailing-robot:indigo-ros-tutorials -f ./Dockerfile_indigo-ros-tutorials ../
docker build -t jamak9/sailing-robot:indigo -f ./Dockerfile_indigo-sailing-robot ../
docker build -t jamak9/sailing-robot -f ./Dockerfile_indigo-sailing-robot ../

