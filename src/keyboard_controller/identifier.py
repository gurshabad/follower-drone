#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import cv
import numpy as np
from sensor_msgs.msg import Image

def segment_colour(frame):
    hsv_roi =  cv2.cvtColor(frame, cv2.cv.CV_BGR2HSV)
    mask_1 = cv2.inRange(hsv_roi, np.array([160, 160,10]), np.array([190,255,255]))
    ycr_roi=cv2.cvtColor(frame,cv2.cv.CV_BGR2YCrCb)
    mask_2=cv2.inRange(ycr_roi, np.array((0.,165.,0.)), np.array((255.,255.,255.)))

    mask = mask_1 | mask_2
    kern_dilate = np.ones((8,8),np.uint8)
    kern_erode  = np.ones((3,3),np.uint8)
    mask= cv2.erode(mask,kern_erode)
    mask=cv2.dilate(mask,kern_dilate)
    return mask

def find_blob(blob):
    largest_contour=0
    cont_index=0
    contours, hierarchy = cv2.findContours(blob, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    for idx, contour in enumerate(contours):
        area=cv2.contourArea(contour)
        if (area > largest_contour):
            largest_contour=area
            cont_index=idx
    r=(0,0,2,2)
    if len(contours) > 0:
        r = cv2.boundingRect(contours[cont_index])

    return r




def callback():
    global xval,yval,xsig,ysig,sig1,sig2,wval,wsig,x_velocity,y_velocity,alt
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        cap.open()




    rate = rospy.Rate(1)
    while True:
        ret ,cvFrame = cap.read()
        mask_red=segment_colour(cvFrame)
        loct=find_blob(mask_red)
        x,y,w,h=loct
        width=max(int(w),int(h))
        distance=(569*18.1)/width
        #18.1 is the diameter of the ball in centimetres. Change this if you are using a different red coloured object, 569 is the focal length of front camera you should be able to find this in ardrone_autonomy package under data camera info
        cv2.rectangle(cvFrame, (x, y), (x + w, y + h), 255, 2)
        cv2.circle(cvFrame, (int(center.x), int(center.y)), 3, (0, 110, 255), -1)
        cv2.imshow('img2',cvFrame)
        #cv2.imshow('img3',cvFrame2)
        #rate.sleep()
        k = cv2.waitKey(5) & 0xff
        if k == 27:
            break
    cv2.destroyAllWindows()
    cap.release()

def identifier():
    rospy.init_node('identifier', anonymous=True)
    callback()
    #rospy.Subscriber('/camera/image', Image, callback)
    rospy.Subscriber('/ardrone/image_raw', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        identifier()
    except rospy.ROSInterruptException:
        pass

