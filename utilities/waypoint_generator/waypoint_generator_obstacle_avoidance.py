#!/usr/bin/python2
# READY FOR MIT
# Script to generate waypoints for the obstacle avoidance challenge
# man:
#   waypoint_generator path/to/waypoint.yaml

from __future__ import print_function

import os
import sys

my_dir = os.path.dirname(__file__)
robot_src_dir = os.path.abspath(os.path.join(my_dir, '../../src/sailing_robot/src'))
sys.path.append(robot_src_dir)

import yaml
import numpy as np
from sailing_robot.navigation import Navigation

# See http://www.dmap.co.uk/utmworld.htm to find the right zone
UTM_ZONE = 30

# Load yaml file given in argument
input_file = sys.argv[1]
#with open(input_file, 'r') as f:
#    yaml_data = yaml.safe_load(f)

output_file = input_file[:-5] + "_gen_obstacle.yaml"

margin = 10 # [m]

#wp1 = yaml_data['wp/table']['wp1']
#wp2 = yaml_data['wp/table']['wp2']
#wp3 = yaml_data['wp/table']['wp3']
#wp4 = yaml_data['wp/table']['wp4']
# course 1
wp1 = (50.82082570829222,-1.311508136900023)
wp2 = (50.820720237801176,-1.31363073133094)
wp4 = (50.820646434491884,-1.3114859407167054)

# course 2
wp1 = (50.82086875660889,-1.3136046333659215)
wp2 = (50.822136584453986,-1.3128785101937752)
wp4 = (50.820930082615554,-1.313871497810471)

nav = Navigation(utm_zone=UTM_ZONE)
wp1_utm = nav.latlon_to_utm(wp1[0], wp1[1])
wp2_utm = nav.latlon_to_utm(wp2[0], wp2[1])
#wp3_utm = nav.latlon_to_utm(wp3[0], wp3[1])
wp4_utm = nav.latlon_to_utm(wp4[0], wp4[1])


# Unit vector 12
v12 = np.array([wp2_utm[0] - wp1_utm[0], wp2_utm[1] - wp1_utm[1]])
d12 = np.linalg.norm(v12)
v12_unit = v12 / d12



# Unit vector 14
v14 = np.array([wp4_utm[0] - wp1_utm[0], wp4_utm[1] - wp1_utm[1]])
d14 = np.linalg.norm(v14)
v14_unit = v14 / d14


# Coordinates of waypoints (see scheme in the wiki
utm_wp = {}
utm_wp['A'] = wp1_utm -margin*v12_unit + 0.5*v14
utm_wp['B'] = wp1_utm + v12/6 + 0.5*v14
utm_wp['B1'] = wp1_utm + v12/6 - 1.5*v14
utm_wp['C'] = wp1_utm + v12/3 + 0.5*v14 
utm_wp['C1'] = wp1_utm + v12/3 - 1.5*v14
utm_wp['D'] = wp1_utm + 0.5*v12 + 0.5*v14
utm_wp['D1'] = wp1_utm + 0.5*v12 - 1.5*v14
utm_wp['E'] =  wp2_utm - v12/3 + 0.5*v14
utm_wp['E1'] =  wp2_utm - v12/3 - 1.5*v14
utm_wp['F'] = wp2_utm - v12/6 + 0.5*v14
utm_wp['F1'] = wp2_utm - v12/6 - 1.5*v14
utm_wp['G'] = wp2_utm + margin*v12_unit + 0.5*v14


def to_wp(wp):
    latlon = nav.utm_to_latlon(wp[0], wp[1])
    return [float(latlon.lat), float(latlon.lon) ]

# Convert new waypoints to lat/long pairs
latlon_wp = {k: to_wp(v) for (k,v) in utm_wp.items()}
yaml_data = {}
yaml_data['wp/table'] = latlon_wp

# Task order
yaml_data['wp/tasks'] = [{'kind': 'to_waypoint', 'waypoint': 'A'},
                        {'kind': 'to_waypoint', 'waypoint': 'B'},
                        {'kind': 'obstacle_waypoints', 'normal': 'C', 'obstacle': 'B1'},
                        {'kind': 'obstacle_waypoints', 'normal': 'D', 'obstacle': 'C1'},
                        {'kind': 'obstacle_waypoints', 'normal': 'E', 'obstacle': 'D1'},
                        {'kind': 'obstacle_waypoints', 'normal': 'F', 'obstacle': 'E1'},
                        {'kind': 'to_waypoint', 'waypoint': 'G'},
                        {'kind': 'to_waypoint', 'waypoint': 'F'},
                        {'kind': 'obstacle_waypoints', 'normal': 'E', 'obstacle': 'F1'},
                        {'kind': 'obstacle_waypoints', 'normal': 'D', 'obstacle': 'E1'},
                        {'kind': 'obstacle_waypoints', 'normal': 'C', 'obstacle': 'D1'},
                        {'kind': 'obstacle_waypoints', 'normal': 'B', 'obstacle': 'C1'},
                        {'kind': 'to_waypoint', 'waypoint': 'A'},
                        ]

with open(output_file, 'w') as f:
    yaml.dump(yaml_data, f)

print(yaml.dump(yaml_data))
print()
print('Written to:', output_file)
