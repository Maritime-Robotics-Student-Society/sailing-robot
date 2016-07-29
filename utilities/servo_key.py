#!/usr/bin/env python


# 2015-04-10
# Public Domain

import time
import curses
import atexit

import pigpio 

SERVO = 24

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

stdscr = curses.initscr()
curses.noecho()
curses.cbreak()

atexit.register(cleanup) # Ensure original screen state is restored.

in_escape = False
in_cursor = False

pulsewidth = MID_PW

pi.set_servo_pulsewidth(SERVO, pulsewidth)

while True:

   time.sleep(0.01)

   c = getch()

   if c == QUIT:
      break

   pw = pulsewidth

   if c == HOME:
      pw = MID_PW # Stop.
   elif c == UP_ARROW:
      pw = MAX_PW # Fastest clockwise.
   elif c == DOWN_ARROW:
      pw = MIN_PW # Fastest anti-clockwise
   elif c == LEFT_ARROW:
      pw = pw - 5 # Shorten pulse.
      if pw < MIN_PW:
         pw = MIN_PW
   elif c == RIGHT_ARROW:
      pw = pw + 5 # Lengthen pulse.
      if pw > MAX_PW:
         pw = MAX_PW

   if pw != pulsewidth:
      pulsewidth = pw
      print("Current PWM %d" % pulsewidth)
      pi.set_servo_pulsewidth(SERVO, pulsewidth)
