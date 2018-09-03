#!/usr/bin/python2
# Script to generate waypoints for the area scanning challenge
# man:
#   waypoint_generator path/to/waypoint.yaml
from __future__ import print_function

import os
import sys
import yaml
from LatLon import LatLon
from pyproj import Proj
import numpy as np

utm_zone = 30

projection = Proj(proj='utm', zone=utm_zone, ellps='WGS84')

def latlon_to_utm(lat, lon):
    """Returns (x, y) coordinates in metres"""
    return projection(lon, lat)

def utm_to_latlon(x, y):
    """Returns a LatLon object"""
    lon, lat = projection(x, y, inverse=True)
    return LatLon(lat, lon)





# my_dir = os.path.dirname(__file__)
# robot_src_dir = os.path.abspath(os.path.join(my_dir, '../../src/sailing_robot/src'))
# sys.path.append(robot_src_dir)

# from sailing_robot.navigation import Navigation

# Load yaml file given in argument
# input_file = sys.argv[1]
# with open(input_file, 'r') as f:
#     yaml_data = yaml.safe_load(f)

output_file = "area_scanning_autogen.yaml"

cellsize = 4 #m

wpA = (50.8210729998,-1.31465664793)
wpB = (50.8222207292,-1.31285527881)
wpC = (50.8196023064,-1.31232136805)
wpD = (50.82075,-1.31052)


wpA_utm = latlon_to_utm(wpA[0], wpA[1])
wpB_utm = latlon_to_utm(wpB[0], wpB[1])
wpC_utm = latlon_to_utm(wpC[0], wpC[1])
wpD_utm = latlon_to_utm(wpD[0], wpD[1])

subX = 45
subY = 58

# Unit vector CD
vCD = np.array([wpD_utm[0] - wpC_utm[0], wpD_utm[1] - wpC_utm[1]])
CD = np.linalg.norm(vCD)
vCD = vCD/CD

# Unit vector CA
vCA = np.array([wpA_utm[0] - wpC_utm[0], wpA_utm[1] - wpC_utm[1]])
CA = np.linalg.norm(vCA)
vCA = vCA/CA


wp_list = [wpC_utm + vCD/2 + vCA/2]

top_wp = (wpC_utm + vCD*(subX-0.5)*cellsize + vCA*(45-0.5)*cellsize)
wp_list.append(top_wp)
dir = -1
for j in range(45):
    wp_list.append(wp_list[-1] - vCA*cellsize)
    for i in range(int(subX/3)):
        wp_list.append(wp_list[-1] + dir*vCD*cellsize*3)
    dir = -dir
        
wp_list.pop(0) # to remove the first wp (only there because it was easier to loop like that...)

# wp_list_top = [wpA_utm - vAB/2 + vAB_orth/2]
# dir = 1
# for wp_idx_vert in range(4):
#     wp_list_top.append(wp_list_top[-1] + vAB)
#     for wp_idx_hor in range(7):
#         wp_list_top.append(wp_list_top[-1] + dir*vAB_orth)
#     dir = -dir
# wp_list_top.pop(0) # to remove the first wp (only there because it was easier to loop like that...)


#### BOTTOM 4x4 part
# wp_list_bot = [wpA_utm + vAB*3.5 + vAB_orth*4.5]
# dir = 1
# for wp_idx_vert in range(4):
#     wp_list_bot.append(wp_list_bot[-1] + vAB)
#     for wp_idx_hor in range(3):
#         wp_list_bot.append(wp_list_bot[-1] + dir*vAB_orth)
#     dir = -dir
# wp_list_bot.pop(0) # to remove the first wp (only there because it was easier to loop like that...)


#### Finish line
# wp_finish = [wpA_utm + vAB*5 + vAB_orth/2, wpA_utm + vAB*5 - vAB_orth/2]
# wp_finish = [wpD_utm - vAB + vAB_orth/2, wpD_utm - vAB - vAB_orth/2] # uncomment if wpD is correctly set
# wp_finish = [wpC_utm + vAB + vAB_orth/2, wpC_utm + vAB - vAB_orth/2] # uncomment if wpC is correctly set


# wp_list = wp_start + wp_list_top + wp_list_bot + wp_finish 

wp_latlon_list = [ utm_to_latlon(wp[0], wp[1]) for wp in wp_list ]

wp_table = {}
wp_tasks = []
idx = 1
for wp in wp_latlon_list:
    wp_table[str(idx)] = [float(wp.lat), float(wp.lon)]
    wp_tasks.append({"kind": "to_waypoint", "waypoint": str(idx)})
    [float(wp.lat), float(wp.lon)]
    idx += 1

#wp_idx_list = [ str(i+1) for i in range(idx-1)]

#yaml_data['wp/list'] = wp_idx_list
yaml_data = {}
yaml_data['wp/tasks'] = wp_tasks
yaml_data['wp/table'] = wp_table

yaml_data['wp/table']['A'] = list(wpA)
yaml_data['wp/table']['B'] = list(wpB)
yaml_data['wp/table']['C'] = list(wpC)
yaml_data['wp/table']['D'] = list(wpD)

with open(output_file, 'w') as f:
    yaml.dump(yaml_data, f)

print(yaml.dump(yaml_data))
print()
print('Written to:', output_file)
