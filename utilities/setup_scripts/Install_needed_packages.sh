#!/bin/sh 
set -e
cd ~

# Install Geos headers (for shapely), and scipy (for compass calibration), install pip too
sudo apt-get --assume-yes install libgeos-dev python-scipy python-pip

# Install:
# Latlon, shapely, pyproj (navigation)
# pynmea2 (reading GPS)
# spidev (Needed for serial)
# tornado (web server for HTML dashboard)
# ina219 library (voltmeter/currentmeter)
yes | sudo pip install Latlon shapely pyproj pynmea2 spidev tornado pi_ina219

# Install vim & bc (calculator)
sudo apt-get --assume-yes install vim bc

# Set time zone to england (it is the same for portugal)
sudo timedatectl set-timezone Europe/London

# pigpio - for talking to GPIO pins (including daemon service)
sudo apt-get --assume-yes install pigpio python-pigpio
sudo cp pigpio.service /lib/systemd/system
sudo systemctl enable pigpio
sudo systemctl start pigpio

cd ~
