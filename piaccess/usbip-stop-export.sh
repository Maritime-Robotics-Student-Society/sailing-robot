#!/bin/bash
# Stop exporting the Xsens USB device over IP.
# This is called from usbip-bind.sh on Tabarly.
set -e

USBIP_DIR=~/raspi-linux/linux-raspberrypi-kernel_1.20161215-1/tools/usb/usbip/src

BUSID=$(cat /tmp/xsens_busid)
echo "Unbinding bus ID: $BUSID"
sudo $USBIP_DIR/usbip unbind -b $BUSID
rm -f /tmp/xsens_busid
