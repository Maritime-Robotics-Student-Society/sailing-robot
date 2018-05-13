#!/usr/bin/python
"""Calibrate the compass MinIMU.

This version of the calibration does not account for tilt; it assumes the boat
stays level.
"""

import os.path
import time
import rospy
import sys
import yaml


defaultboatname = 'blackpython'
print("Name of the calibration file ["+ defaultboatname +"]")
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
LGD = 0x6b #Device I2C slave address
LSM = 0x1d #Device I2C slave address

imu = ImuReader(IMU_BUS, LSM, LGD)
imu.check_status()
imu.configure_for_reading()

data_X = []
data_Y = []
data_Z = []

for i in range(300):
    try:
        #Read data from the chips ----------------------
        time.sleep(0.1)
        magx, magy, magz = imu.read_mag_field()

        data_X.append(magx)
        data_Y.append(magy)
        data_Z.append(magz)

        minx = min(data_X)
        miny = min(data_Y)
        minz = min(data_Z)
        maxx = max(data_X)
        maxy = max(data_Y)
        maxz = max(data_Z)
        
        print
        print(minx)
        print(maxx)
        print(miny)
        print(maxy)
        print(minz)
        print(maxz)
        
    except KeyboardInterrupt:
        print("Interrupted")
        break



minx = min(data_X)
miny = min(data_Y)
minz = min(data_Z)

maxx = max(data_X)
maxy = max(data_Y)
maxz = max(data_Z)

offset_X = (maxx + minx)/2
offset_Y = (maxy + miny)/2
offset_Z = (maxz + minz)/2

range_X = maxx - minx
range_Y = maxy - miny
range_Z = maxz - minz

# rospy.set_param('/calibration/compass', {'XOFFSET': offset_X,
#                                           'YOFFSET': offset_Y,
#                                           'ZOFFSET': offset_Z,
#                                           'XSCALE': range_X,
#                                           'YSCALE': range_Y,
#                                           'ZSCALE': range_Z,
#                                           })



print("XOFFSET = " + str(offset_X))
print("YOFFSET = " + str(offset_Y))
print("ZOFFSET = " + str(offset_Z))
print("XSCALE = " + str(range_X))
print("YSCALE = " + str(range_Y))
print("ZSCALE = " + str(range_Z))



# Load of the previous configuration in the configuration file
# (we are just updating it without overwriting everything)
if os.path.isfile(calibration_file):
    calib_dict = yaml.load(open(calibration_file))
else:
    calib_dict = {}


print("-----------------------------------------")
print("Great, now dumping to file: " + filename)
calib_dict['compass'] ={'XOFFSET': offset_X,
                        'YOFFSET': offset_Y,
                        'ZOFFSET': offset_Z,
                        'XSCALE': range_X,
                        'YSCALE': range_Y,
                        'ZSCALE': range_Z,
                        }

with open(calibration_file, 'w') as calib_file:
    yaml.dump(calib_dict, calib_file, default_flow_style=False)




