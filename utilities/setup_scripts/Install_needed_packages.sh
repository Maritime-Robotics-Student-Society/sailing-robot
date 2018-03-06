#!/bin/sh 
cd ~

# Install Geos headers (for shapely), and scipy (for compass calibration), install pip too
sudo apt-get --assume-yes install libgeos-dev python-scipy python-pip

# Install Latlon, shapely (navigation), pynmea2 (reading GPS)
yes | sudo pip install Latlon shapely pynmea2

# Needed for serial
yes | sudo pip install spidev


# Install pyproj (cartographic projections)
yes | sudo pip install pyproj

# Install vim
sudo apt-get --assume-yes install vim

# Install bc
sudo apt-get --assume-yes install bc


# Needed for dashboard
yes | sudo pip install tornado

# Set time zone to england (it is the same for portugal)
sudo timedatectl set-timezone Europe/London

# pigpio - for talking to GPIO pins (including daemon service)
sudo apt-get --assume-yes install pigpio python-pigpio
sudo cp pigpio.service /lib/systemd/system
sudo systemctl enable pigpio
sudo systemctl start pigpio

# Install ina219 library (voltmeter/currentmeter)
sudo pip install pi_ina219


cd ~
