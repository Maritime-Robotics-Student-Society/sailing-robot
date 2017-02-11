#!/bin/bash
# This script is useful when using Tabarly as a ros server
# It will create a wifi network (called Tabarly), and run roscore
# People then connect to it via ssh: ssh -Y soton-pirate@192.168.12.
# And can use their own ros environement
# 
# Once done, simply hit ctrl+c to kill ros and stop the wifi network


trap ctrl_c INT
function ctrl_c() {
	echo Cleaning up
	sudo create_ap --stop $wifi_interface
	killall roscore
	sleep 1
}


# Detection of the wifi interface
wifi_interface=$(ifconfig | grep wlan | grep -v wlan0 | awk '{print $1}' | tail -n 1)


echo "
Tell people to connect to Tabarly wifi (the small wifi adapter is needed)
SSID: Tabarly
Password: pirates!

Then they have to ssh to tabarly via: 
ssh -Y soton-pirate@192.168.12.1
Password: pirates!"

# Creation of the access point
sudo create_ap --daemon --no-virt $wifi_interface wlan0 Tabarly 'pirates!' > /dev/null

echo 'running roscore'
roscore & > /dev/null
echo
echo "hit ctrl c to kill"
sleep 50000



