#!/bin/bash

# Send the time from this computer to the raspi

# This is not suitable for setting a precise time: it makes no effort to account
# for the delay between reading the local clock and setting it on the raspi.
# But when the raspi is powered on, its clock can easily be days out, so even
# setting an approximate time is important.

# See also utilities/setclock, which sets the clock from a GPS signal

set -e

echo "Sending time to the pi at ${SAIL_PI_IP:=192.168.12.1}..."
TIMESTAMP=$(date --utc +%Y-%m-%dT%H:%M:%S)

set -x
ssh -t pi@$SAIL_PI_IP "sudo date --utc +%Y-%m-%dT%H:%M:%S -s $TIMESTAMP"
