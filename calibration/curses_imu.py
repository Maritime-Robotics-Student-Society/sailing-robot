#!/usr/bin/python
"""Display raw readings from the MinIMU in a curses terminal interface.

Run this as a script to play back readings stored in a CSV file.
The curses interface defined here is also used in other scripts in this folder.
"""
import csv
from curses import wrapper
import math
import os
import sys
import time

class Extrema(object):
    def __init__(self):
        self.min = 99999
        self.max = -99999

    def update(self, value):
        if value < self.min:
            self.min = value
        if value > self.max:
            self.max = value
        return self.min, value, self.max

class Extrema3D(object):
    def __init__(self):
        self.x = Extrema()
        self.y = Extrema()
        self.z = Extrema()

class IMUDisplay(object):
    def __init__(self, stdscr):
        self.stdscr = stdscr
        stdscr.clear()

        stdscr.addstr(0, 0, '      |   min  |   now  |   max ')
        stdscr.addstr(1, 0, '    x |')
        stdscr.addstr(2, 0, 'mag y |')
        stdscr.addstr(3, 0, '    z |')
        stdscr.addstr(5, 0, '    x |')
        stdscr.addstr(6, 0, 'acc y |')
        stdscr.addstr(7, 0, '    z |')
        stdscr.addstr(9, 0, 'pitch |')
        stdscr.addstr(10,0, ' roll |')

        self.mag_extreme = Extrema3D()
        self.acc_extreme = Extrema3D()
        self.pitch_extreme = Extrema()
        self.roll_extreme = Extrema()

    def update_mag(self, x, y, z):
        self.stdscr.addstr(1, 8, '{:6d} | {:6d} | {:6d}'.format(*self.mag_extreme.x.update(x)))
        self.stdscr.addstr(2, 8, '{:6d} | {:6d} | {:6d}'.format(*self.mag_extreme.y.update(y)))
        self.stdscr.addstr(3, 8, '{:6d} | {:6d} | {:6d}'.format(*self.mag_extreme.z.update(z)))

    def update_acc(self, x, y, z):
        self.stdscr.addstr(5, 8, '{:6d} | {:6d} | {:6d}'.format(*self.acc_extreme.x.update(x)))
        self.stdscr.addstr(6, 8, '{:6d} | {:6d} | {:6d}'.format(*self.acc_extreme.y.update(y)))
        self.stdscr.addstr(7, 8, '{:6d} | {:6d} | {:6d}'.format(*self.acc_extreme.z.update(z)))

    def update_pitch_roll(self, pitch, roll):
        self.stdscr.addstr(9, 8, '{:6.1f} | {:6.1f} | {:6.1f}'.format(*self.pitch_extreme.update(pitch)))
        self.stdscr.addstr(10,8, '{:6.1f} | {:6.1f} | {:6.1f}'.format(*self.roll_extreme.update(roll)))

def pitch_roll(acc_x, acc_y, acc_z):
    pitch_r = math.atan2(acc_x, math.sqrt(acc_y**2 + acc_z**2))
    roll_r = math.atan2(-acc_y, -acc_z)
    return math.degrees(pitch_r), math.degrees(roll_r)

def main(stdscr):
    display = IMUDisplay(stdscr)

    with open(sys.argv[1]) as f:
        cr = csv.DictReader(f)
        for row in cr:
            x, y, z = int(row['mag_x']), int(row['mag_y']), int(row['mag_z'])
            display.update_mag(x, y, z)
            x, y, z = int(row['acc_x']), int(row['acc_y']), int(row['acc_z'])
            display.update_acc(x, y, z)
            display.update_pitch_roll(*pitch_roll(x, y, z))
            stdscr.refresh()
            time.sleep(0.1)

    stdscr.getkey()

if __name__ == '__main__':
    if len(sys.argv) < 2 or not os.path.isfile(sys.argv[1]):
        sys.exit('Usage: curses_imu.py calibration_file.csv')

    try:
        wrapper(main)
    except KeyboardInterrupt:
        print('^C')
