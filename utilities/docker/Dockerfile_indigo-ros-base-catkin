FROM ros:indigo-ros-base
Maintainer Martin Biggs

USER root
RUN apt-get update \
 && apt-get upgrade --assume-yes \
 && apt-get install --assume-yes g++ vim openssh-server \
 && mkdir /var/run/shhd \
 && useradd --user-group --create-home --shell /bin/bash pi \
 && echo 'pi:raspberry' | chpasswd \
 && echo "pi ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers.d/pi \
 && rm -fr /var/lib/apt/lists

USER pi
RUN bash -c "\
# Fix for Tab AutoCompletion not working in ROS
        echo 'export LC_ALL=\"C\"' >> ~/.bashrc \
# Fix 'WARNING: Terminal is not fully functional'
     && echo 'export TERM=xterm' >> ~/.bashrc \
     && echo 'source /opt/ros/indigo/setup.bash' >> ~/.bashrc \
     && source /opt/ros/indigo/setup.bash \
     && mkdir -p ~/catkin_ws/src \
     && cd ~/catkin_ws/src \
     && catkin_init_workspace \
     && cd ~/catkin_ws/ \
     && catkin_make \
     && rosdep update \
     && echo 'source /home/pi/catkin_ws/devel/setup.bash' >> ~/.bashrc"
