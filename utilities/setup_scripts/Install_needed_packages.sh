#!/bin/sh 
set -e

# Install:
# Geos headers (for shapely)
# scipy (for compass calibration)
# pip (Python packages)
# vim
# bc (calculator)
# i2c-tools (for i2cdetect)
# gpsd-clients (simple GPS debug)
echo "Installing apt packages..."
sudo apt-get --assume-yes install libgeos-dev python-scipy python-pip vim bc i2c-tools gpsd-clients


# Install:
# Latlon, shapely, pyproj (navigation)
# pynmea2 (reading GPS)
# spidev (Needed for serial)
# tornado (web server for HTML dashboard)
# ina219 library (voltmeter/currentmeter)
echo "Installing Python packages..."
yes | sudo pip install --upgrade pip
yes | sudo pip install Latlon shapely pyproj pynmea2 spidev tornado pi_ina219

# Increase UART frequency
echo "Setting UART frequency..."
echo "enable_uart=1" | sudo tee -a /boot/config.txt > /dev/null

# Set time zone to england (it is the same for portugal)
echo "Setting timezone..."
sudo timedatectl set-timezone Europe/London

# pigpio - for talking to GPIO pins (including daemon service)
echo "Installing pigpio from apt..."
sudo apt-get --assume-yes install pigpio python-pigpio
echo "Installing and starting pigpio service..."
sudo cp pigpio.service /lib/systemd/system
sudo systemctl enable pigpio
sudo systemctl start pigpio
echo "Done."
