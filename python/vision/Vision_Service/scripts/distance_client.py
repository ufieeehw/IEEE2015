#!/usr/bin/env python
import tf
import sys
import rospy
from random import randint
from ieee2015_msgs.srv import ComputeDistance
from geometry_msgs.msg import Point
#from std_msgs import float64
class myNode:
    def __init__(self, *args):
        self.tf = TransformListener()


    def some_method(self):

        if self.tf.frameExists('/course') and self.tf.frameExists('/end_camera'):
            t = self.tf.getLatestCommonTime('/course', '/end_camera')
            listener = tf.TransformListener()
    #read_end_camera()
    #rospy.Subscriber('/tf', Point, callback)


            (trans , rot) = listener.lookupTransform('/course', '/end_camera', t)
            #global end_camera_height 
            #end_camera_height = 4.0           
            #end_camera_height=trans[2]



def compute_distance_client(point1, point2):
    print "Waiting for service"
    rospy.wait_for_service('compute_distance')
    print "Found Service"
    try:
        handle_two_points = rospy.ServiceProxy('compute_distance', ComputeDistance)
        #end_camera_height=4.0
        resp1 = handle_two_points(point1, point2, 0.5 )
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [point1 point2]"%point





if __name__ == "__main__":
    point =  [randint(2,9), randint(2,9), 0]
    pointa =  [randint(2,9), randint(2,9), 0]
    if len(point) == 3:
        point1 = Point(*point)
        point2 = Point(*pointa)
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s ----> %s"%(point1, point2)
    print "%s ----> %s = %s"%(point1, point2, compute_distance_client(point1, point2))
