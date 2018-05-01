#!/bin/bash
# Connect to the Xsens on the Raspi using USB-over-IP.
# Run this from Tabarly while connected to the raspi by wifi.
set -e

sudo modprobe vhci-hcd
echo "SSH-ing to Pi to set up usbip export..."
ssh -t pi@${SAIL_PI_IP:=192.168.12.1} bash sailing-robot/piaccess/usbip-export.sh
BUSID=$(ssh pi@${SAIL_PI_IP} cat /tmp/xsens_busid)

echo "Attaching [-r $SAIL_PI_IP -b $BUSID]"
sudo usbip attach -r $SAIL_PI_IP -b $BUSID

echo
echo "Now use Xsens on this computer. Press return here to disconnect."
read

echo "Detaching"
sudo usbip detach -p 0

echo "SSHing to Pi to unbind Xsens"
ssh -t pi@${SAIL_PI_IP} bash sailing-robot/piaccess/usbip-stop-export.sh
