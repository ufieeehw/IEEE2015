#!/usr/bin/env python
import tf
import sys
import rospy
#import random #for generating random pixels for testing
import cv2
import numpy as np
#from random import randint #for generating random pixels for testing
from ieee2015_msgs.srv import ComputeDistance
from geometry_msgs.msg import Point            #Pixel Locations
from geometry_msgs.msg import PointStamped     #Vector components in /robot frame (x,y,z)
''' This client is serviced by distance_server. It provides the service the center point of the camera frame(point1) and a desired pixel coordinate. 
    These pixel coordinates are stored as the geometry_msgs.msg Point.
    
'''

def compute_distance_client(point1, point2):#point1 should be center point of the camera view 
    print "Waiting for service"             #point2 is the desired pixel location(were you want the camera to be postitoned) 
    rospy.wait_for_service('compute_distance')
    print "Found Service"
    try:
        handle_two_points = rospy.ServiceProxy('compute_distance', ComputeDistance)
        response = handle_two_points(point1, point2)
        return response                     #Vector components of desired location in robot frame
    except rospy.ServiceException, e:       
        print "Service call failed: %s"%e    
'''
def usage():                                #staged for deleting
    return "%s [point1 point2]"%point
'''
def getColor(image):
    img = image

    img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)#HSV is good for color detection with lighting flux  

    '''
    values derived from experimenting with the trackbars.py to 
    opitimze for 4 colors present on simon says
    '''
    #thresholding and building heat mask
    h1 = 25
    s1 = 85
    v1 = 55
    h2 = 254
    s2 = 254
    v2 = 200
    lower_value = np.array([h1, s1, v1], np.uint8)
    upper_value = np.array([h2, s2, v2], np.uint8)
    #endthresholding

    temp = np.array(0)
    mask = cv2.inRange(img, lower_value, upper_value)
    bImg = cv2.bitwise_or(mask, temp)
    #kernel = np.ones((150,150),np.uint8)
    #bImg = cv2.morphologyEx(bImg, cv2.MORPH_CLOSE, kernel)#messed something up was included in a tutorial not entirely required here
    ret, thresh = cv2.threshold(bImg, 127, 255, 0)
    #end thresholding and building heat mask

    contours = cv2.findContours(thresh, 1, 2)#findimg contours really just masses of colors
    cnt = contours[0]                        #selecting main contour
    M = cv2.moments(cnt)                     #moment of contour http://docs.opencv.org/modules/imgproc/doc/structural_analysis_and_shape_descriptors.html?highlight=moments#moments

    if M['m00'] < 200:                       #Filters out noise when no real souce of color is present
        return 1 
    else:  
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        target = (cx,cy,0.2)                        #contour centroid
        cv2.line(img,(320,240),(cx,cy),(255,0,0),10)#Draw a blue line from what I hope is the center of the image to the cetroid of the contour
        point2=Point(*target)                       #convert target in the form of a list to point2 in the form of a geometry message Point
        cv2.imshow("Blue Line Image", img)          #display
        return point2



#if __name__ == "__main__":
    
def main(image):
    point =  [320, 240, 0] #list
    #pointa =  [randint(200,409), randint(222,409), random.random()/4] #for generating random pixels for testing
    point1 = Point(*point)  #convert list to Point Msg
    point2 = getColor(image) #Is a point Msg
    if (point2 == 1): #This is in case there is no target point in the view 
        return        #should really only be used for the getColor function when trying to find the center of any avalible color
                      #It keeps the program from crashing when it doenst have any distance to compute
    '''
    if len(point) != 3: #staged for deleting
        print usage()
        sys.exit(1)
    '''
    print "Requesting %s ----> %s"%(point1, point2)
    print "%s ----> %s = %s"%(point1, point2, compute_distance_client(point1, point2))
    return compute_distance_client(point1, point2) #PointStamped in robot frame for comanding arm 