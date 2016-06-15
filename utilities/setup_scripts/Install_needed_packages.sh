#!/bin/sh 
cd ~

# Install Geos headers (for shapely)
sudo apt-get --assume-yes intall libgeos-dev

# Install Latlon, shapely (navigation) and pynmea2 (reading GPS)
yes | sudo pip install Latlon shapely pynmea2

# Needed for serial
yes | sudo pip install spidev

# Install vim
sudo apt-get --assume-yes install vim
cd ~
