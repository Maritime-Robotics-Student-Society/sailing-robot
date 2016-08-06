#!/bin/sh 
cd ~

# Install Geos headers (for shapely)
sudo apt-get --assume-yes install libgeos-dev

# Install Latlon, shapely (navigation) and pynmea2 (reading GPS)
yes | sudo pip install Latlon shapely pynmea2

# Needed for serial
yes | sudo pip install spidev

# Wiringpi2 needed for hardware pwm
yes | sudo pip install wiringpi2

# Install vim
sudo apt-get --assume-yes install vim

# Install bc
sudo apt-get --assume-yes install bc

# Install sharply for navigation node
yes | sudo pip install shapely

# Needed for dashboard
yes | sudo pip install tornado

# Set time zone to england (it is the same for portugal)
sudo timedatectl set-timezone Europe/London

# How to install pigpio on raspberry pi
#rm pigpio.zip
#sudo rm -rf PIGPIO
#wget abyz.co.uk/rpi/pigpio/pigpio.zip
#unzip pigpio.zip
#cd PIGPIO
#make -j4
#sudo make install

cd ~
