#!/bin/bash
# Exports the Xsens USB device over IP.
# This is called from usbip-bind.sh on Tabarly.
set -e

echo $USER

USBIP_DIR=~/raspi-linux/linux-raspberrypi-kernel_1.20161215-1/tools/usb/usbip/src
XSENS_USBID="2639:0300"

# Get the bus ID for the Xsens
BUSID=$($USBIP_DIR/usbip list --parsable --local | awk -F# "\$2 ~/$XSENS_USBID/ { sub(/busid=/, \"\"); print \$1}")
echo "Bus ID: $BUSID"

# Start the daemon
sudo $USBIP_DIR/usbipd -D
sleep 0.2  # Give daemon a chance to start

# Bind the device
sudo $USBIP_DIR/usbip bind -b $BUSID
echo $BUSID > /tmp/xsens_busid
