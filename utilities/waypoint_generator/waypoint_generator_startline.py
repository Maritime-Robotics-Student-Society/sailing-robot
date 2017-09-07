#!/usr/bin/python2
# READY FOR MIT
# Script to generate waypoints for the obstacle avoidance challenge
# man:
#   waypoint_generator path/to/waypoint.yaml

from __future__ import print_function

import sys
import yaml
import numpy as np
from sailing_robot.navigation import Navigation

# Load yaml file given in argument
input_file = sys.argv[1]
with open(input_file, 'r') as f:
    yaml_data = yaml.safe_load(f)

output_file = input_file[:-5] + "_gen_obstacle.yaml"

margin = 10 # [m]

wp1 = yaml_data['wp/table']['1']
wp0 = yaml_data['wp/table']['0']
#wp3 = yaml_data['wp/table']['wp3']


nav = Navigation()
wp1_utm = nav.latlon_to_utm(wp1[0], wp1[1])
wp0_utm = nav.latlon_to_utm(wp0[0], wp0[1])


# Unit vector 10
v10 = np.array([wp0_utm[0] - wp1_utm[0], wp0_utm[1] - wp1_utm[1]])
d10 = np.linalg.norm(v10)
v10_unit = v10 / d10
print(d10)


# Unit vector orth to 10
v10_orth = np.array([-v10_unit[1], v10_unit[0]])



# Coordinates of waypoints (see scheme in the wiki
f1 = wp1_utm + v10_unit*d10/2 - v10_orth*margin 
f2 = wp1_utm + v10_unit*d10/2 + v10_orth*margin 

def to_wp(wp):
    latlon = nav.utm_to_latlon(wp[0], wp[1])
    return [float(latlon.lat), float(latlon.lon) ]


f1 = to_wp(f1)
f2 = to_wp(f2)

print("f1:", f1, ",")
print("f2:", f2, ",")
