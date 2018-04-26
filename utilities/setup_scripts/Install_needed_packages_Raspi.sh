#!/bin/bash 
set -e

## Swap file for the pi
# Install needed packages
sudo apt-get --assume-yes install dphys-swapfile

# Copy default dphys-swapfile
sudo cp /etc/dphys-swapfile /etc/dphys-swapfile.old

# Delete old and create new dphys-swapfile
sudo rm /etc/dphys-swapfile
sudo touch /etc/dphys-swapfile

# Contents of dphys-swapfile
sudo echo "CONF_SWAPSIZE=1024" >> /etc/dphys-swapfile

# Increase swap size
sudo dphys-swapfile setup
sudo dphys-swapfile swapon



# i2c-tools (for i2cdetect)
# gpsd-clients (simple GPS debug)
echo "Installing apt packages..."
sudo apt-get --assume-yes install i2c-tools gpsd-clients

# spidev (Needed for serial)
# ina219 library (voltmeter/currentmeter)
yes | sudo pip install spidev pi_ina219


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
