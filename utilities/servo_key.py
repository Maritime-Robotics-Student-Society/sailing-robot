#!/usr/bin/env python
'''Servo testing script.

Run this as:
    ./servo_key.py

Press:

- Left/right to increase/decrease rudder pulse (pin 13)
- Up/down to increase/decrease sail pulse (pin 24)
- Home to jump to starting positions
- Q to quit.
'''

# 2015-04-10
# Public Domain

from __future__ import print_function

import argparse
import time
import curses
import atexit
import sys

import pigpio

RUDDER_PIN = 13
SAIL_PIN = 24

RUDDER_START = 1500
SAIL_START = 1250

RUDDER_STEP = 100
SAIL_STEP = 5

MIN_PW = 1000
MID_PW = 1500
MAX_PW = 2000

NONE        = 0
LEFT_ARROW  = 1
RIGHT_ARROW = 2
UP_ARROW    = 3
DOWN_ARROW  = 4
HOME        = 5
QUIT        = 6

def getch():
   global in_escape, in_cursor
   c = stdscr.getch()

   key = NONE

   if c == 27:
      in_escape = True
      in_cursor = False
   elif c == 91 and in_escape:
      in_cursor = True
   elif c == 68 and in_cursor:
      key = LEFT_ARROW
      in_escape = False
   elif c == 67 and in_cursor:
      key = RIGHT_ARROW
      in_escape = False
   elif c == 65 and in_cursor:
      key = UP_ARROW
      in_escape = False
   elif c == 66 and in_cursor:
      key = DOWN_ARROW
      in_escape = False
   elif c == 72 and in_cursor:
      key = HOME
      in_escape = False
   elif c == 113 or c == 81:
      key = QUIT
   else:
      in_escape = False
      in_cursor = False

   return key

def cleanup():
   curses.nocbreak()
   curses.echo()
   curses.endwin()
   pi.stop()

pi = pigpio.pi()

def interact():
    global stdscr
    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()

    atexit.register(cleanup) # Ensure original screen state is restored.

    c = getch()

    print("Rudder pin {} / Sail pin {}".format(RUDDER_PIN, SAIL_PIN), end='\r\n')
    print('Left/Right: rudder', end='\r\n')
    print('Up/down: sail', end='\r\n')
    print('Home: reset', end='\r\n')
    print('Q: quit', end='\r\n')

    in_escape = False
    in_cursor = False

    rudder_pw = RUDDER_START
    sail_pw = SAIL_START

    pi.set_servo_pulsewidth(SAIL_PIN, sail_pw)
    pi.set_servo_pulsewidth(RUDDER_PIN, rudder_pw)

    while True:

       time.sleep(0.01)

       if c == QUIT:
          break

       if c == HOME:
          rudder_pw = MID_PW   # Reset
       elif c == UP_ARROW:
          sail_pw = min(MAX_PW, sail_pw + SAIL_STEP)
       elif c == DOWN_ARROW:
          sail_pw = max(MIN_PW, sail_pw - SAIL_STEP)
       elif c == LEFT_ARROW:
          rudder_pw = max(MIN_PW, rudder_pw - RUDDER_STEP)
       elif c == RIGHT_ARROW:
          rudder_pw = min(MAX_PW, rudder_pw + RUDDER_STEP)

       print("Rudder PWM {} / Sail PWM {}".format(rudder_pw, sail_pw), end='\r\n')
       pi.set_servo_pulsewidth(RUDDER_PIN, rudder_pw)
       pi.set_servo_pulsewidth(SAIL_PIN, sail_pw)

       c = getch()

if __name__ == '__main__':
    interact()
