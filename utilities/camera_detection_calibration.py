#!/usr/bin/python
# READY FOR MIT

import rospy
import sys
import math
import time

import numpy as np
import cv2

Lower = np.array( rospy.get_param('camera_detection/Lower_color'))
Upper = np.array( rospy.get_param('camera_detection/Upper_color'))
Lower_hsv1 = np.array( rospy.get_param('camera_detection/Lower_color_hsv1'))
Upper_hsv1 = np.array( rospy.get_param('camera_detection/Upper_color_hsv1'))
Lower_hsv2 = np.array( rospy.get_param('camera_detection/Lower_color_hsv2'))
Upper_hsv2 = np.array( rospy.get_param('camera_detection/Upper_color_hsv2'))
threshold = rospy.get_param('camera_detection/threshold')


#camera = cv2.VideoCapture(0) 
#(bool, image) = camera.read()
image = cv2.imread(sys.argv[1])
image_size = image.shape[0]*image.shape[1]

print 'Calibration procedure for the camera'
print 'use up/j or down/k to change the threshold (=% of the image that is expected to be in the range of colors'

files = sys.argv[1:]
for file in files:
    try: 
        print ''
        print '--------------------------------------'
        print file
        print ''
        image = cv2.imread(file)
        if bool:
            #cv2.imwrite('image.png',image)
            mask = cv2.inRange(image, Lower, Upper)
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            mask_hsv1 = cv2.inRange(hsv, Lower_hsv1, Upper_hsv1)
            mask_hsv2 = cv2.inRange(hsv, Lower_hsv2, Upper_hsv2)
             
            percent_detect1 = 1.0*cv2.countNonZero(mask_hsv1)  / image_size # percentage of the image that contains the expected colors
            print percent_detect1 
            percent_detect2 = 1.0*cv2.countNonZero(mask_hsv2)  / image_size # percentage of the image that contains the expected colors
            print percent_detect2 
            
            print percent_detect2 + percent_detect1 
            if percent_detect1+percent_detect2  >= threshold: 
                print 'detected'
            else:
                print 'nothing detected'
            
            print 'threshold: ' + str(threshold)

            masked_image = cv2.bitwise_and(image, image, mask=mask_hsv1 )
            cv2.imshow('image', masked_image)
            key = cv2.waitKey()

            if key == 65362 or key == ord('k'): # up arrow key of k
                threshold = threshold + 0.01
            elif key == 65364 or key == ord('j'):# down arrow key of j
                threshold = threshold - 0.01


    except KeyboardInterrupt:
        print("Interrupted")
        break

print 'the threshold is: ' + str(threshold)
