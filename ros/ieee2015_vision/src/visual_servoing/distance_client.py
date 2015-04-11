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
        target = (cx,cy,0.06)                        #contour centroid
        cv2.line(img,(320,240),(cx,cy),(255,0,0),10)#Draw a blue line from what I hope is the center of the image to the cetroid of the contour
        point2=Point(*target)                       #convert target in the form of a list to point2 in the form of a geometry message Point
        cv2.imshow("Blue Line Image", img)          #display
        return point2

        
def card_pick_up(flag):
    if (flag == None):
        return hover_over_card(1)
    if (flag == hover1):
        return drop()
    if (flag== dropped):
        return hover_over_card(2)
       
def hover_over_card(iteration):
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans_diff,rot_diff) = listener.lookupTransform('/arm_camera','/card_picker', rospy.Time(0)) #change to /end_camera frame 
            diff_x=trans_diff[0]
            diff_y=trans_diff[1]
            diff_height = trans_diff[2]
            (trans,rot) = listener.lookupTransform('/robot','/arm_camera', rospy.Time(0)) #change to /end_camera frame 
            end_camera_x=trans[0]
            end_camera_y=trans[1]
            end_camera_height = trans[2]

            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    xcomp2= end_camera_x + diff_x
    ycomp2= end_camera_y + diff_y
    zcomp2= end_camera_z + diff_z
    dist1 = (xcomp2, ycomp2, zcomp2) 
    dist2 = Point(*dist1)           #convert from list to Point msg
    #print "dist", dist
    #print "Returning vector components [%s , %s to %s , %s = %s]"%(req.point1.x, req.point1.y, req.point2.x, req.point2.y, (dist))
    
    
    dist = PointStamped()               #make a point stamped to send to the arm ontroller
    dist.header.stamp = rospy.Time.now()
    dist.header.frame_id = '/robot'
    dist.point = dist2
    if (iteration == 1):
        return dist, "ready"
    if(iteration == 2):
        return dist 
def drop():
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/robot','/card_picker', rospy.Time(0)) #change to /end_camera frame 
            card_picker_x=trans[0]
            card_picker_y=trans[1]
            card_picker_z = trans[2]
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

    xcomp2= card_picker_x
    ##
    dist1 = (xcomp2, ycomp1, 0.005) 
    dist2 = Point(*dist1)  
    dist = PointStamped()               #make a point stamped to send to the arm ontroller
    dist.header.stamp = rospy.Time.now()
    dist.header.frame_id = '/robot'
    dist.point = dist2
    #flag = dropped
    return dist  #, flag



#if __name__ == "__main__":
    
def main(image, mission):

    point =  [320, 240, 0.05] #list
    #pointa =  [randint(200,409), randint(222,409), random.random()/4] #for generating random pixels for testing
    point1 = Point(*point)  #convert list to Point Msg
    #if conditions based on mission state

    point2 = getColor(image) #Is a point Msg
    
    if (abs(point2.point.x - point1.point.x) <= 15 and abs(point2.point.y - point1.point.x <= 15)):
        if (mission==cards):
            return "pick"       
    

    if (point2 == 1): #This is in case there is no target point in the view 
        return        #should really only be used for the getColor function when trying to find the center of any avalible color
                      #It keeps the program from crashing when it doenst have any distance to compute

    print "Requesting %s ----> %s"%(point1, point2)
    print "%s ----> %s = %s"%(point1, point2, compute_distance_client(point1, point2))
    return compute_distance_client(point1, point2) #PointStamped in robot frame for comanding arm 