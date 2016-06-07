#!/bin/sh 
cd ~

# Install Latlon and pynmea2 for gps
yes | sudo pip install Latlon pynmea2

# Needed for serial
yes | sudo pip install spidev

# Install vim
sudo apt-get --assume-yes install vim
cd ~
