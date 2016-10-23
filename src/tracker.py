#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.ticker as ticker
from tracking.msg import Posinfo

bridge = CvBridge()
pub = ()
#kalman constants for distance KF below
actual_variance_in_system = 0.1
error_in_readings = 0.1
x0 = 0.0
x0_bar = 1.0
x1 = 0.0
x1_bar = 1.0
x2 = 0.0
x2_bar = 1.0
corrected_x, corrected_y, corrected_distance = 0,0,0

def distanceKF(distance):
    global x0, x0_bar
    measurement = distance
    x0_estimated = x0
    x0_estimated_error = x0_bar + actual_variance_in_system
    curve_moving_factor = x0_estimated_error / (x0_estimated_error + actual_variance_in_system)
    x0 = x0_estimated + actual_variance_in_system * (measurement - x0_estimated)
    x0_bar = (1 - actual_variance_in_system) * x0_estimated_error
    return x0

def xKF(xc):
    global x1, x1_bar
    measurement = xc
    x1_estimated = x1
    x1_estimated_error = x1_bar + actual_variance_in_system
    curve_moving_factor = x1_estimated_error / (x1_estimated_error + actual_variance_in_system)
    x1 = x1_estimated + actual_variance_in_system + (measurement - x1_estimated)
    x1_bar = (1 - actual_variance_in_system) * x1_estimated_error
    return x1


def yKF(yc):
    global x2, x2_bar
    measurement = yc
    x2_estimated = x2
    x2_estimated_error = x2_bar + actual_variance_in_system
    curve_moving_factor = x2_estimated_error / (x2_estimated_error + actual_variance_in_system)
    x2 = x2_estimated + actual_variance_in_system + (measurement - x2_estimated)
    x2_bar = (1 - actual_variance_in_system) * x2_estimated_error
    return x2

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
    if len(contours) > 0: r = cv2.boundingRect(contours[cont_index])
    return r

def callback(data):
    global pub, corrected_x, corrected_y, corrected_distance
    try:
        cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)

    mask_red=segment_colour(cv_image)
    loct=find_blob(mask_red)
    x,y,w,h=loct
    width=max(int(w),int(h))
    distance=(569*18.1)/width
    cv2.rectangle(cv_image, (x, y), (x + w, y + h), 255, 2)
    cv2.circle(cv_image, ((x + x + w)/2,(y + y + h)/2), 3, (0, 110, 255), -1)

    #Kalman filter calls
    if distance < 4000:
        corrected_distance = distanceKF(distance)
        corrected_x = xKF((x+x+w)/2)
        corrected_y = yKF((y+y+h)/2)
        msg = Posinfo()
        msg.x = corrected_x
        msg.y = corrected_y
        msg.distance = corrected_distance
        pub.publish(msg)

    else:
        corrected_distance = distanceKF(corrected_distance)
        corrected_x = xKF(corrected_x)
        corrected_y = yKF(corrected_y)
        #msg = Posinfo()
        #msg.x = corrected_x
        #msg.y = corrected_y
        #msg.distance = corrected_distance
        #pub.publish(msg)

    rospy.loginfo('I heard %d %d', int(corrected_x), int(corrected_y))
    cv2.circle(cv_image, (int(corrected_x),int(corrected_y)), 3, (0, 255, 0), -2)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)




def tracker():
    global pub
    pub = rospy.Publisher('trackerpub', Posinfo, queue_size=1000)
    rospy.init_node('tracker', anonymous=True)
    rospy.Subscriber('/ardrone/image_raw', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    tracker()
