#!/usr/bin/env python
import tf
import rospy
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
import math
from ieee2015_msgs.srv import ComputeDistance
from geometry_msgs.msg import Point

#from std_msgs import float64

#takes two points and a height of an object
#looks up the transform from world(/course) to the downward facing camera(/end_camera)
#computes the real world distance between the two points by applying a camera specific constant(pixles/meters) derived from experimenting with the camera 
'''
class myNode:
    def __init__(self, *args):
        self.tf = TransformListener()
        return end_camera_height
'''
#global end_camera_height
'''
def some_method():
    if tf.frameExists('/course') and tf.frameExists('/end_camera'):
        t = tf.getLatestCommonTime('/course', '/end_camera')
        listener = tf.TransformListener()
        #read_end_camera()
        #rospy.Subscriber('/tf', Point, callback)


        (trans , rot) = listener.lookupTransform('/course', '/end_camera', t)
        #global end_camera_height 
        #end_camera_height = 4.0           
        #end_camera_height=trans[2]
        end_camera_height=trans[2]
        print "end_camera_height", end_camera_height
        return end_camera_height
'''
def handle_two_points(req):
    xcomp=(1.0/84000)(0.2-0.1)(req.point1.x - req.point2.x)
    ycomp=(1.0/84000)(0.2-0.1)(req.point1.y - req.point2.y)
    #dist = pixeldist*(fake_tfv[2]-0.1)*(1.0/84000.0)
    dist = (xcomp, ycomp)
    dist = Point(*dist)
    print "dist", dist
    print "Returning vector components [%s , %s to %s , %s = %s]"%(req.point1.x, req.point1.y, req.point2.x, req.point2.y, (dist))
    return (dist)


def compute_distance_server():
    rospy.init_node('compute_distance_server')
    s = rospy.Service('compute_distance', ComputeDistance, handle_two_points)
    print "Ready to compute distance."
    rospy.spin()

if __name__== "__main__":
    compute_distance_server()


