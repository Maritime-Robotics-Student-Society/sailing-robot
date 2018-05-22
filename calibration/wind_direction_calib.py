#!/usr/bin/python
"""Calibrate the magnetic field sensors which read the wind vane.

Two steps:
- Turn wind vane around to get min/max magnetic field readings
- Point vane to bow of boat to correct angle
"""

import os.path
import time, math
import rospy
import socket
import sys
import yaml


defaultboatname = 'blackpython'

print("Name of the calibration file ["+ defaultboatname +"]")
print("(hit enter to use default)")
boatname = raw_input("")

if not boatname:
    boatname = defaultboatname

filename = "calibration_"+ boatname + ".yaml"

my_dir = os.path.dirname(__file__)
robot_src_dir = os.path.abspath(os.path.join(my_dir, '../src/sailing_robot/src'))
sys.path.append(robot_src_dir)
calibration_file = os.path.join(robot_src_dir, '../launch/parameters/', filename)



from sailing_robot.imu_utils import ImuReader


IMU_BUS = 1
LSM = 0x1e #Device I2C slave address
LGD = 0x6a #Device I2C slave address
data_X = []
data_Y = []

    # try:    # Check we can talk to ROS before trying to calibrate
    #     rospy.get_param('/rosversion')
    # except socket.error:
    #     sys.exit("Can't connect to parameter server. Ensure roscore is running.")

imu = ImuReader(IMU_BUS, LSM, LGD)
imu.check_status()
imu.configure_for_reading()

for i in range(300):
    try:
        #Read data from the chips ----------------------
        time.sleep(0.1)
        _, magy, magz = imu.read_mag_field()

        data_X.append(magy)
        data_Y.append(magz)

        minx = min(data_X)
        miny = min(data_Y)
        maxx = max(data_X)
        maxy = max(data_Y)
        print
        print(minx)
        print(maxx)
        print(miny)
        print(maxy)
    except KeyboardInterrupt:
        print("Interrupted")
        break



minx = min(data_X)
miny = min(data_Y)

maxx = max(data_X)
maxy = max(data_Y)

Xoffset = (maxx + minx)/2
Yoffset = (maxy + miny)/2

Xscale = maxx - minx
Yscale = maxy - miny

print("Now hold still the wind vane in direction of the bow, and press enter when ready")
raw_input()

_, magy, magz = imu.read_mag_field()

magy = (magy - Xoffset) * Xscale
magz = (magz - Yoffset) * Yscale
angle_offset = math.atan2(magy, magz)*(180/math.pi) % 360


print("Formated output in case file save failed, you shouldn't need that:")
print("wind_dir: {"+ 
      "ANGLEOFFSET: "+ str(angle_offset) + 
      ", XOFFSET: " + str(Xoffset) + 
      ", XSCALE: " + str(Xscale) + 
      ", YOFFSET: "+ str(Yoffset)+
      ", YSCALE: "+str(Yscale)+"}")



# Load of the previous configuration from the configuration file
# (we are just updating it without overwriting everything)
if os.path.isfile(calibration_file):
    calib_dict = yaml.load(open(calibration_file))
else:
    calib_dict = {}


print("-----------------------------------------")
print("Great, now dumping to file: " + filename)
calib_dict['wind_dir'] = {'XOFFSET': Xoffset,
                          'YOFFSET': Yoffset,
                          'XSCALE': Xscale,
                          'YSCALE': Yscale,
                          'ANGLEOFFSET': angle_offset,
                          }

with open(calibration_file, 'w') as calib_file:
    yaml.dump(calib_dict, calib_file, default_flow_style=False)

