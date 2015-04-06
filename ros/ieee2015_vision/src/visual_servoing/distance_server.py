#!/usr/bin/env python
import tf
import rospy
import geometry_msgs.msg
import math
from ieee2015_msgs.srv import ComputeDistance
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point


#takes two points and a height of an object
####point1 #center point (0,0,0)
####point2 #featured point (x,y,h) h=hight of feature in meters
#looks up the transform from world(/course) to the downward facing camera(/end_camera)
#computes the real world distance between the two points by applying a camera specific constant(pixles/meters) derived from experimenting with the camera 



def handle_two_points(req):
    '''
        Real_World_width = constant * distance_camera_to_object * Pixel_width

    Calculate real world coordinates of desired camera location in the robot frame
    (1.0/840) is a constant calculated from experimenting with the camera
    distance_camera_to_object = end_camera_height(in robot frame)+ 0.21658(vertical translation from course to robot)-req.point2.z(height of object)
    '''
    xcomp1=((end_camera_x)+(1.0/840)*(end_camera_height+0.21658-req.point2.z)*(req.point2.x - req.point1.x))
    ycomp1=((end_camera_y)+(1.0/840)*(end_camera_height+0.21658-req.point2.z)*(req.point2.y - req.point1.y))
    dist1 = (xcomp1, ycomp1, 0.12) 
    dist2 = Point(*dist1)           #convert from list to Point msg
    #print "dist", dist
    #print "Returning vector components [%s , %s to %s , %s = %s]"%(req.point1.x, req.point1.y, req.point2.x, req.point2.y, (dist))
    
    
    dist = PointStamped()               #make a point stamped to send to the arm ontroller
    dist.header.stamp = rospy.Time.now()
    dist.header.frame_id = '/robot'
    dist.point = dist2
    #dist = (header.frame_id, header.stamp, point)
    print dist
    return dist


def compute_distance_server():
    #rospy.init_node('compute_distance_server')
    s = rospy.Service('compute_distance', ComputeDistance, handle_two_points)
    print "Ready to compute distance."
    #rospy.spin()

if __name__== "__main__":
    #global end_camera_height
    #global end_camera_x
    #global end_camera_y

    compute_distance_server()
    rospy.init_node('ieee2015_tf_broadcaster2')#I think this is wrong
    
    #For getting current camera position
    
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/wrist_joint','/robot', rospy.Time(0)) #change to /end_camera frame 
            end_camera_x=trans[0]
            end_camera_y=trans[1]
            end_camera_height = trans[2]
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        #print end_camera_height

        rate.sleep()

    


#Meaningless Change