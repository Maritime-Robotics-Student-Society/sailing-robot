#!/usr/bin/python2
# READY FOR MIT
# Script to generate waypoints to round buoys
# man:
#   waypoint_generator path/to/waypoint.yaml

from __future__ import print_function

import sys
import yaml
import numpy as np
from sailing_robot.navigation import Navigation


RADIUS = 2    # [m]
CLOCKWISE = False


# Load yaml file given in argument
input_file = sys.argv[1]
with open(input_file, 'r') as f:
    yaml_data = yaml.safe_load(f)

output_file = input_file[:-5] + "_gen_round.yaml"


# Generate the list of buoys
BUOY_LIST = []
wp_table = {}
id_wp = 1
for idx in yaml_data['wp/list']:
    BUOY_LIST.append(yaml_data['wp/table'][idx])

    wp_table['b' + str(id_wp)] = yaml_data['wp/table'][idx] # to keep buoys in the final waypoint table
    id_wp += 1



def leg_wp(wpA, wpB):
    '''
        Generate 2 waypoints on the leg between wpA and wpB
    '''
    nav = Navigation()
    wpA_utm = nav.latlon_to_utm(wpA[0], wpA[1])
    wpB_utm = nav.latlon_to_utm(wpB[0], wpB[1])

    vAB = np.array([wpB_utm[0] - wpA_utm[0], wpB_utm[1] - wpA_utm[1]])
    AB = np.linalg.norm(vAB)

    if CLOCKWISE:
        vAB_orth = np.array([-vAB[1], vAB[0]])/AB
    else:
        vAB_orth = np.array([vAB[1], -vAB[0]])/AB

    wpA_p_utm = wpA_utm + vAB_orth*RADIUS
    wpB_p_utm = wpB_utm + vAB_orth*RADIUS

    wpA_p = nav.utm_to_latlon(wpA_p_utm[0], wpA_p_utm[1])
    wpB_p = nav.utm_to_latlon(wpB_p_utm[0], wpB_p_utm[1])

    return wpA_p, wpB_p 


n = len(BUOY_LIST)
BUOY_LIST.append(BUOY_LIST[0])

waypoint_list = []
for idx in range(n):
    wpA_p, wpB_p = leg_wp(BUOY_LIST[idx], BUOY_LIST[idx+1])
    waypoint_list.append(wpA_p)
    waypoint_list.append(wpB_p)


idx = 1
for wp in waypoint_list:
    wp_table[str(idx)] =  [float(wp.lat), float(wp.lon)]
    idx += 1

wp_list = [ str(i+1) for i in range(idx-1)]

yaml_data['wp/list'] = wp_list
yaml_data['wp/table'] = wp_table

with open(output_file, 'w') as f:
    yaml.dump(yaml_data, f)

print(yaml.dump(yaml_data))
print()
print('Written to:', output_file)
