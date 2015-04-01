#!/usr/bin/env python
import tf
import rospy
import geometry_msgs.msg
import math
from ieee2015_msgs.srv import ComputeDistance
from geometry_msgs.msg import Point


#takes two points and a height of an object
####point1 #center point (0,0,0)
####point2 #featured point (x,y,h) h=hight of feature in meters
#looks up the transform from world(/course) to the downward facing camera(/end_camera)
#computes the real world distance between the two points by applying a camera specific constant(pixles/meters) derived from experimenting with the camera 



def handle_two_points(req):
    xcomp1=(1.0/84000)*(end_camera_height-req.point2.z)*(req.point1.x - req.point2.x)
    ycomp1=(1.0/84000)*(end_camera_height-req.point2.z)*(req.point1.y - req.point2.y)
    dist1 = (xcomp1, ycomp1, 0)
    dist = Point(*dist1)
    #print "dist", dist
    #print "Returning vector components [%s , %s to %s , %s = %s]"%(req.point1.x, req.point1.y, req.point2.x, req.point2.y, (dist))
    return (dist)


def compute_distance_server():
    rospy.init_node('compute_distance_server')
    s = rospy.Service('compute_distance', ComputeDistance, handle_two_points)
    print "Ready to compute distance."
    rospy.spin()

if __name__== "__main__":
    rospy.init_node('tf_ieee2015')

    listener = tf.TransformListener()


    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/course','/wrist_joint', rospy.Time(0))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        global end_camera_height
        end_camera_height = trans[2]
        #print end_camera_height

        rate.sleep()

    compute_distance_server()


