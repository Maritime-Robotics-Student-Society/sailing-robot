#!/usr/bin/env python
'''Servo testing script.

Run this as:
    ./servo_wiggle.py
'''

# 2015-04-10
# Public Domain

from __future__ import print_function

import time
import pigpio

RUDDER_PIN = 13
SAIL_PIN = 24

MIN_PW = 1000
MID_PW = 1500
MAX_PW = 2000

STEP = 50   # Change pw by STEP
SLEEP = 0.1  # every SLEEP seconds

pi = pigpio.pi()

pw = MIN_PW
rising = True

while True:
    time.sleep(SLEEP)
    if pw >= MAX_PW:
       rising = False
       print('max', end=' ')
    elif pw <= MIN_PW:
       rising = True
       print('min', end=' ')

    if rising:
        pw += STEP
    else:
        pw -= STEP

    pi.set_servo_pulsewidth(RUDDER_PIN, pw)
    pi.set_servo_pulsewidth(SAIL_PIN, pw)
