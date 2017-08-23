#!/usr/bin/python
# Licence MIT

# import rospy
import sys
import math
import time

import numpy as np
import cv2

print 'Calibration procedure for the camera'
print
print 'If working on image files, add their path as parameter'
print 'The number at the to gives the current threshold'
print 'Click on the buoy you want to detect untill it is entirely colored'
print 'Press "q" to exit'
print 'Press "u" to undo the last color pick'
print 'Press "n" to move to the next picture (if using files)'
print 'Press "p" to move to the previous picture (if using files)'
print

# change the camera id to 1 if using a usb camera
cameraId = 0


colors = []


def on_mouse_click (event, x, y, flags, frame):
    if event == cv2.EVENT_LBUTTONUP:
        colors.append(frame[y,x].tolist())

isRed = False 

Lower = np.array([0,0,0])
Upper = np.array([0,0,0])
Lower_hsv1 = np.array([0, 0, 0])
Upper_hsv1 = np.array([0, 0, 0])
Lower_hsv2 = np.array([0, 0, 0])
Upper_hsv2 = np.array([0, 0, 0])

if len(sys.argv)>1:
    print "Using image files"
    delay = 250
    usingCamera = False
    currImage = 0
    ImageFileList = sys.argv[1:]
    nbImage = len(ImageFileList)
else:
    print "Using webcam " + str(cameraId)
    delay = 30
    usingCamera = True
    camera = cv2.VideoCapture(cameraId) 


while True:

    if usingCamera:
        (_, image) = camera.read()
    else:
        imageFile = ImageFileList[currImage]
        image = cv2.imread(imageFile)

    image_size = image.shape[0]*image.shape[1]

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask_hsv1 = cv2.inRange(hsv, Lower_hsv1, Upper_hsv1)
    mask_hsv2 = cv2.inRange(hsv, Lower_hsv2, Upper_hsv2)
     
    percent_detect1 = 1.0*cv2.countNonZero(mask_hsv1) / image_size # percentage of the image that contains the expected colors
    percent_detect2 = 1.0*cv2.countNonZero(mask_hsv2) / image_size # percentage of the image that contains the expected colors
    
    percent_detect = percent_detect1 + percent_detect2

    masked_image = cv2.bitwise_and(image, image, mask=mask_hsv1 + mask_hsv2)

    # print of the % of the image detected at the top left
    cv2.putText(image, "{:.4f} /1".format(percent_detect), (10, 50), cv2.FONT_HERSHEY_PLAIN, 2, (0, 0, 255), 2)
    cv2.imshow('frame', image/2 + masked_image/2)
    cv2.setMouseCallback('frame', on_mouse_click, hsv)

    if colors:
        if isRed:
            Lower[0] = max(c[0] if c[0]<90 else 0 for c in colors)
            Upper[0] = min(c[0] if c[0]>=90 else 180 for c in colors)
        else:
            Lower[0] = min(c[0] for c in colors)
            Upper[0] = max(c[0] for c in colors)
            # if color both at a low and high H value, we are in around the "red" domain, so using 2 ranges of H values
            if Lower[0]<90 and Upper[0] >=90 and (Upper[0] - Lower[0])> 30:
                isRed = True

        Lower[1] = min(c[1] for c in colors)
        Lower[2] = min(c[2] for c in colors)
        Upper[1] = max(c[1] for c in colors)
        Upper[2] = max(c[2] for c in colors)

        if isRed:
            Lower_hsv1 = np.array([0, Lower[1], Lower[2]])
            Upper_hsv1 = np.array([Lower[0], 255, 255])
            Lower_hsv2 = np.array([Upper[0], Lower[1], Lower[2]])
            Upper_hsv2 = np.array([185, 255, 255])
        else:
            Lower_hsv1 = np.array([Lower[0], Lower[1], Lower[2]])
            Upper_hsv1 = np.array([Upper[0], 255, 255])
            Lower_hsv2 = np.array([0, 0, 0])
            Upper_hsv2 = np.array([0, 0, 0])

#         print "----------------------------------"
#         print "Current HSV " + str(colors[-1])
#         print "Lower       " + str(Lower)
#         print "Upper       " + str(Upper)
#         print 
#         print "Lower1      " + str(Lower_hsv1)
#         print "Upper1      " + str(Upper_hsv1)
#         print "Lower2      " + str(Lower_hsv2)
#         print "Upper2      " + str(Upper_hsv2)


    key = cv2.waitKey(delay)
    if key == ord('n') and not usingCamera:
        if currImage >= nbImage-1:
            currImage = 0
        else:
            currImage += 1
 
    if key == ord('p') and not usingCamera:
        if currImage == 0:
            currImage = nbImage-1
        else:
            currImage -= 1

    if key == ord('q'):
        break

    if key == ord('u') and colors:
        del colors[-1]

#     if key == ord('r'):
#         isRed = not isRed
#         print isRed
           

print
print "copy/paste these lines in your parameter file"
print
print "################################################################"
print """# Lower color 1:
camera_detection/Lower_color_hsv1: [""" + ",".join(map(str,Lower_hsv1)) + """]
# Upper color 1:
camera_detection/Upper_color_hsv1: [""" +  ",".join(map(str,Upper_hsv1)) + "]"
print
print """# Lower color 2:
camera_detection/Lower_color_hsv2: [""" +  ",".join(map(str,Lower_hsv2)) + """]
# Upper color 2:
camera_detection/Upper_color_hsv2: [""" + ",".join( map(str,Upper_hsv2)) + "]"
print "################################################################"

