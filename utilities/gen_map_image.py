#!/usr/bin/python2

import sys
import os
import smopy
import yaml

from PIL import Image
import numpy as np
from sailing_robot.navigation import Navigation


my_dir = os.path.dirname(__file__)
image_dir = os.path.abspath(os.path.join(my_dir, 'map_bg_images'))

# Load yaml file given in argument
input_file = sys.argv[1]
with open(input_file, 'r') as f:
    yaml_data = yaml.safe_load(f)


waypoints = np.array(yaml_data['wp/table'].values())

origin = [waypoints[:,0].mean(), waypoints[:,1].mean()]

side_dist=250 # half of the size of the square in m

location = os.path.basename(input_file).split("_")[0]

output_file_name = os.path.join(image_dir,
                                str(origin[0]) + '_' +
                                str(origin[1]) + "_" + 
                                str(side_dist) + "_" + 
                                location + ".png")

nav = Navigation()
origin_utm = nav.latlon_to_utm(origin[0], origin[1])

minx = - side_dist
maxx = + side_dist
miny = - side_dist
maxy = + side_dist

SO_corner = nav.utm_to_latlon(origin_utm[0] + minx, origin_utm[1] + miny)
NE_corner = nav.utm_to_latlon(origin_utm[0] + maxx, origin_utm[1] + maxy)

image_map = smopy.Map((float(SO_corner.lat),
                       float(SO_corner.lon),
                       float(NE_corner.lat),
                       float(NE_corner.lon)), z=18, maxtiles=302, margin=0)

mapminx, mapminy = image_map.to_pixels((float(SO_corner.lat),
                                        float(SO_corner.lon)), )

mapmaxx, mapmaxy = image_map.to_pixels((float(NE_corner.lat),
                                        float(NE_corner.lon)), )

mapminx = int(mapminx)
mapminy = int(mapminy)
mapmaxx = int(mapmaxx)
mapmaxy = int(mapmaxy)
image = image_map.to_numpy()[mapmaxy:mapminy, mapminx:mapmaxx, :]

im = Image.fromarray(image)
im.save(output_file_name)


