FROM sotonsailbot/ros:indigo-desktop-catkin
Maintainer Martin Biggs

USER root
RUN apt-get --assume-yes install libgeos-dev python-scipy python-pip \
    && pip install --upgrade pip \
    && pip install Latlon shapely pynmea2 spidev wiringpi2 jinja2 \
         folium "tornado<5" pandas
# Fix for Mac's looking for python in /usr/local/bin
RUN ln -s /usr/bin/python /usr/local/bin/python

USER pi
RUN bash -c "echo 'source /home/pi/sailing-robot/devel/setup.bash' >> ~/.bashrc"
RUN bash -c "echo 'cd ~' >> ~/.bashrc"
RUN bash -c "echo 'sudo service ssh start' > ~/start_ssh_server"
RUN bash -c "chmod u+x ~/start_ssh_server"
