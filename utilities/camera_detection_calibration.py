#!/usr/bin/python
# READY FOR MIT

import rospy
import math
import time

import numpy as np
import cv2

Lower = np.array( rospy.get_param('camera_detection/Lower_color'))
Upper = np.array( rospy.get_param('camera_detection/Upper_color'))
threshold = rospy.get_param('camera_detection/threshold')


camera = cv2.VideoCapture(0) 
(bool, image) = camera.read()
image_size = image.shape[0]*image.shape[1]

print 'Calibration procedure for the camera'
print 'use up/j or down/k to change the threshold (=% of the image that is expected to be in the range of colors'


while not rospy.is_shutdown():
    try: 
        print ''
        print '--------------------------------------'
        print ''
        (bool, image) = camera.read()
        if bool:
            mask = cv2.inRange(image, Lower, Upper)

            percent_detect = 1.0*cv2.countNonZero(mask)  / image_size # percentage of the image that contains the expected colors
            
            if percent_detect >= threshold:
                print 'detected'
            else:
                print 'nothing detected'
            
            print 'threshold: ' + str(threshold)

            masked_image = cv2.bitwise_and(image, image, mask=mask)
            cv2.imshow('image', masked_image)
            key = cv2.waitKey(0)

            if key == 65362 or key == ord('k'): # up arrow key of k
                threshold = threshold + 0.01
            elif key == 65364 or key == ord('j'):# down arrow key of j
                threshold = threshold - 0.01




    except KeyboardInterrupt:
        print("Interrupted")
        break

print 'the threshold is: ' + str(threshold)
