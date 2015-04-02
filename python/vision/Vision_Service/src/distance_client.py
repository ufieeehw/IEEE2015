#!/usr/bin/env python
import tf
import sys
import rospy
import random 
from random import randint
from ieee2015_msgs.srv import ComputeDistance
from geometry_msgs.msg import Point



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





if __name__ == "__main__":
    point =  [randint(2,9), randint(2,9), 0]
    pointa =  [randint(200,409), randint(222,409), random.random()]
    point1 = Point(*point)
    point2 = Point(*pointa)
    if len(point) != 3:
        print usage()
        sys.exit(1)
    print "Requesting %s ----> %s"%(point1, point2)
    print "%s ----> %s = %s"%(point1, point2, compute_distance_client(point1, point2))
