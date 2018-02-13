#!/usr/bin/python
"""Monitor the compass MinIMU and display readings using curses.
"""
import curses
import os
import sys
import time

my_dir = os.path.dirname(__file__)
robot_src_dir = os.path.abspath(os.path.join(my_dir, '../src/sailing_robot/src'))
sys.path.append(robot_src_dir)

from sailing_robot.imu_utils import ImuReader
from curses_imu import IMUDisplay, pitch_roll

IMU_BUS = 1
LGD = 0x6a #Device I2C slave address
LSM = 0x1e #Device I2C slave address

imu = ImuReader(IMU_BUS, LSM, LGD)
imu.check_status()
imu.configure_for_reading()


def monitor(stdscr):
    display = IMUDisplay(stdscr)
    stdscr.addstr(curses.LINES-1, 0, 'Press Ctrl-C to quit')
    
    while True:
        time.sleep(0.1)
        
        magx, magy, magz = imu.read_mag_field()
        accx, accy, accz = imu.read_acceleration()

        display.update_mag(magx, magy, magz)
        display.update_acc(accx, accy, accz)
        display.update_pitch_roll(*pitch_roll(accx, accy, accz))
        stdscr.refresh()

try:
    curses.wrapper(monitor)
except KeyboardInterrupt:
    print('Stopped')
