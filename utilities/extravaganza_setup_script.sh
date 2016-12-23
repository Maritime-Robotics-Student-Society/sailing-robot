#!/bin/bash

trap ctrl_c INT
function ctrl_c() {
	echo Cleaning up
	sudo create_ap --stop $wifi_interface
	killall roscore
	sleep 1
}

wifi_interface=$(ifconfig | grep wlan | grep -v wlan0 | awk '{print $1}' | tail -n 1)

# Creation of the access point

echo "
Tell people to connect to Tabarly wifi (the small wifi adapter is needed)
SSID: Tabarly
Password: pirates!

Then they have to ssh to tabarly via: 
ssh -Y soton-pirate@192.168.12.1
Password: pirates!"

sudo create_ap --daemon --no-virt $wifi_interface wlan0 Tabarly 'pirates!' > /dev/null

echo 'running roscore'
roscore & > /dev/null
echo
echo "hit ctrl c to kill"
sleep 50000



