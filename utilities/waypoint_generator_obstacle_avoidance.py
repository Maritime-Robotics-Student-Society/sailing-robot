#!/usr/bin/python2
# READY FOR MIT
# Script to generate waypoints for the obstacle avoidance challenge
# man:
#   waypoint_generator path/to/waypoint.yaml

import sys
import yaml
import numpy as np
from sailing_robot.navigation import Navigation

# Load yaml file given in argument
input_file = sys.argv[1]
yaml_data = yaml.load(file(input_file, 'r'), Loader=yaml.Loader)

output_file = input_file[:-5] + "_gen_obstacle.yaml"

margin = 4 # [m]

wp1 = yaml_data['wp/table']['wp1']
wp2 = yaml_data['wp/table']['wp2']
wp3 = yaml_data['wp/table']['wp3']
wp4 = yaml_data['wp/table']['wp4']


nav = Navigation()
wp1_utm = nav.latlon_to_utm(wp1[0], wp1[1])
wp2_utm = nav.latlon_to_utm(wp2[0], wp2[1])
wp3_utm = nav.latlon_to_utm(wp3[0], wp3[1])
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
wpA = wp1_utm -margin*v12_unit + 0.5*v14
wpB = wp1_utm + (50-margin)*v12_unit + 0.5*v14 
wpC0 = wp1_utm + 0.5*v12 + 0.5*v14
wpC1 = wp1_utm + 0.5*v12 - 0.5*v14
wpD =  wp2_utm -(50-margin)*v12_unit + 0.5*v14
wpE = wp2_utm + margin*v12_unit + 0.5*v14


def to_wp(wp):
    latlon = nav.utm_to_latlon(wp[0], wp[1])
    return [float(latlon.lat), float(latlon.lon) ]

wpA = to_wp(wpA)
wpB = to_wp(wpB)
wpC0 = to_wp(wpC0)
wpC1 = to_wp(wpC1)
wpD = to_wp(wpD)
wpE = to_wp(wpE)


yaml_data['wp/tasks'] = [{'kind': 'to_waypoint', 'waypoint': 'A'},
                        {'kind': 'to_waypoint', 'waypoint': 'B'},
                        {'kind': 'obstacle_waypoints', 'normal': 'C0', 'obstacle': 'C1'},
                        {'kind': 'to_waypoint', 'waypoint': 'D'},
                        {'kind': 'to_waypoint', 'waypoint': 'E'},
                        {'kind': 'to_waypoint', 'waypoint': 'D'},
                        {'kind': 'obstacle_waypoints', 'normal': 'C0', 'obstacle': 'C1'},
                        {'kind': 'to_waypoint', 'waypoint': 'B'},
                        ]

yaml_data['wp/list'] = ['A']

yaml_data['wp/table'] = {}
yaml_data['wp/table']['A'] = wpA
yaml_data['wp/table']['B'] = wpB
yaml_data['wp/table']['C0'] = wpC0
yaml_data['wp/table']['C1'] = wpC1
yaml_data['wp/table']['D'] = wpD
yaml_data['wp/table']['E'] = wpE

yaml.dump(yaml_data, file(output_file, 'w'))
print output_file
print
print yaml.dump(yaml_data)
