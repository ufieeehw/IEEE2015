#!/usr/bin/env python
import tf
import sys
import rospy
import random 
import cv2
import numpy as np
from random import randint
from ieee2015_msgs.srv import ComputeDistance
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped


def compute_distance_client(point1, point2):
    print "Waiting for service"
    rospy.wait_for_service('compute_distance')
    print "Found Service"
    try:
        handle_two_points = rospy.ServiceProxy('compute_distance', ComputeDistance)
        #end_camera_height=4.0
        resp1 = handle_two_points(point1, point2)
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [point1 point2]"%point

def getColor(image):
    img = image

    temp = np.array(0)
    kernel = np.ones((150,150),np.uint8)


    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)#RGB?
    #print 1
    #binary = (img > 35) & (img < 200)
    #print 2 
    #print 2.3
    #print img
    # get current positions of four trackbars
    h1 = 25
    s1 = 85
    v1 = 55

    h2 = 254
    s2 = 254
    v2 = 200

    lower_value = np.array([h1, s1, v1], np.uint8)
    upper_value = np.array([h2, s2, v2], np.uint8)

    mask = cv2.inRange(img, lower_value, upper_value)
    bImg = cv2.bitwise_or(mask, temp)

    #bImg = cv2.morphologyEx(bImg, cv2.MORPH_CLOSE, kernel)

    ret, thresh = cv2.threshold(bImg, 127, 255, 0)

    contours = cv2.findContours(thresh, 1, 2)
    cnt = contours[0]
    M = cv2.moments(cnt)

    if M['m00'] == 0:
        cy = sys.maxint
        cx = sys.maxint
    else:  
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
    target = (cx,cy,0.2)
    cv2.line(img,(0,0),(cx,cy),(255,0,0),10)
    point2=Point(*target)
    cv2.imshow("Blue Line Image", img)
    return point2



#if __name__ == "__main__":
    
def main(image):
    point =  [320, 240, 0]
    pointa =  [randint(200,409), randint(222,409), random.random()/4]
    point1 = Point(*point)
    point2 = getColor(image)
    if len(point) != 3:
        print usage()
        sys.exit(1)
    print "Requesting %s ----> %s"%(point1, point2)
    print "%s ----> %s = %s"%(point1, point2, compute_distance_client(point1, point2))
    return compute_distance_client(point1, point2)