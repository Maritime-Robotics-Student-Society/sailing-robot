#!/bin/sh
# check if pigpiod is running and run it if it is not

if ps ax | grep -v grep | grep pigpiod > /dev/null
then
  echo pigpiod already running
else
  echo launching pigpiod
  sudo pigpiod
fi
