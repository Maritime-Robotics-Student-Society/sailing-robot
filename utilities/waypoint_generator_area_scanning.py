#!/usr/bin/python2
# READY FOR MIT
# Script to generate waypoints for the area scanning challenge
# man:
#   waypoint_generator path/to/waypoint.yaml

import sys
import yaml
import numpy as np
from sailing_robot.navigation import Navigation

# Load yaml file given in argument
input_file = sys.argv[1]
yaml_data = yaml.load(file(input_file, 'r'), Loader=yaml.Loader)

output_file = input_file[:-5] + "_gen_area.yaml"


wpA = yaml_data['wp/table']['A'] # top left (start line)
wpB = yaml_data['wp/table']['B'] # start line (second point)
#wpC = yaml_data['wp/table']['C'] #finish line (top)
wpD = yaml_data['wp/table']['D'] # finish line (second point)
wpE = yaml_data['wp/table']['E'] # top right corner


nav = Navigation()
wpA_utm = nav.latlon_to_utm(wpA[0], wpA[1])
wpB_utm = nav.latlon_to_utm(wpB[0], wpB[1])
#wpC_utm = nav.latlon_to_utm(wpC[0], wpC[1])
wpD_utm = nav.latlon_to_utm(wpD[0], wpD[1])
wpE_utm = nav.latlon_to_utm(wpE[0], wpE[1])


# Unit vector AB
vAB = np.array([wpB_utm[0] - wpA_utm[0], wpB_utm[1] - wpA_utm[1]])/3.0

# Unit vector orth to AB
vAB_orth = np.array([-vAB[1], vAB[0]])
vAE = np.array([wpE_utm[0] - wpA_utm[0], wpE_utm[1] - wpA_utm[1]])/6.0
vAB_orth = vAE

#### Start line
wp_start = [wpA_utm +vAB/2 - vAB_orth/2]


#### TOP 8x4 part
wp_list_top = [wpA_utm - vAB/2 + vAB_orth/2]
dir = 1
for wp_idx_vert in range(3):
    wp_list_top.append(wp_list_top[-1] + vAB)
    for wp_idx_hor in range(5):
        wp_list_top.append(wp_list_top[-1] + dir*vAB_orth)
    dir = -dir
wp_list_top.pop(0) # to remove the first wp (only there because it was easier to loop like that...)


#### BOTTOM 4x4 part
wp_list_bot = [wpA_utm + vAB*2.5 + vAB_orth*5.5]
dir = -1
for wp_idx_vert in range(3):
    wp_list_bot.append(wp_list_bot[-1] + vAB)
    for wp_idx_hor in range(2):
        wp_list_bot.append(wp_list_bot[-1] + dir*vAB_orth)
    dir = -dir
wp_list_bot.pop(0) # to remove the first wp (only there because it was easier to loop like that...)


#### Finish line
wp_finish = [wpA_utm + vAB*5 + vAB_orth/2, wpA_utm + vAB*5 - vAB_orth/2]
wp_finish = [wpD_utm - vAB + vAB_orth/2, wpD_utm - vAB - vAB_orth/2] # uncomment if wpD is correctly set
# wp_finish = [wpC_utm + vAB + vAB_orth/2, wpC_utm + vAB - vAB_orth/2] # uncomment if wpC is correctly set


wp_list = wp_start + wp_list_top + wp_list_bot + wp_finish 

wp_latlon_list = [ nav.utm_to_latlon(wp[0], wp[1]) for wp in wp_list]

wp_table = {}
wp_tasks = []
idx = 1
for wp in wp_latlon_list:
    wp_table[str(idx)] =  [float(wp.lat), float(wp.lon)]
    wp_tasks.append({"kind": "to_waypoint", "waypoint": str(idx)})
    [float(wp.lat), float(wp.lon)]
    idx += 1

#wp_idx_list = [ str(i+1) for i in range(idx-1)]

#yaml_data['wp/list'] = wp_idx_list
yaml_data['wp/tasks'] = wp_tasks
yaml_data['wp/table'] = wp_table

yaml_data['wp/table']['A'] = wpA
yaml_data['wp/table']['B'] = wpB
#yaml_data['wp/table']['C'] = wpC
yaml_data['wp/table']['D'] = wpD
yaml_data['wp/table']['E'] = wpE

yaml.dump(yaml_data, file(output_file, 'w'))
print output_file
print
print yaml.dump(yaml_data)
