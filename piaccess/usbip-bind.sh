#!/bin/bash
# Connect to the Xsens on the Raspi using USB-over-IP.
# Run this from Tabarly while connected to the raspi by wifi.
set -e

PIACCESS_DIR=$(dirname "$BASH_SOURCE[0]")

echo "SSH-ing to Pi to set up usbip export..."
$PIACCESS_DIR/ssh pi@${SAIL_PI_IP:=192.168.12.1} bash ~/sailing-robot/piaccess/usbip-export.sh
BUSID=$(ssh pi@${SAIL_PI_IP} echo /tmp/xsens_busid)

echo "Attaching [-r $SAIL_PI_IP -b $BUSID]"
sudo usbip attach -r $SAIL_PI_IP -b $BUSID

echo
echo "Now use Xsens on this computer. Press Ctrl-C here to disconnect"
sleep 604800 || true

sudo usbip detach -p 0

$PIACCESS_DIR/ssh pi@${SAIL_PI_IP} bash ~/sailing-robot/piaccess/usbip-stop-export.sh 
