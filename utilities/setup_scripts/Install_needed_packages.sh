#!/bin/sh 
set -e

# Install:
# Geos headers (for shapely)
# scipy (for compass calibration)
# pip (Python packages)
# vim
# bc (calculator)
# opencv (vision analisys for obstacle avoidance)
echo "Installing apt packages..."
sudo apt-get --assume-yes install libgeos-dev python-scipy python-pip vim bc python-opencv


# Install:
# Latlon, shapely, pyproj (navigation)
# pynmea2 (reading GPS)
# tornado (web server for HTML dashboard)
echo "Installing Python packages..."
yes | sudo pip install --upgrade pip
yes | sudo pip install Latlon shapely pyproj==1.9.6 pynmea2 spidev tornado 

