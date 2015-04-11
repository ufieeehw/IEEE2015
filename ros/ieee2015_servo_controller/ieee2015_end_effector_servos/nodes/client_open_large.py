#!/usr/bin/env python

import sys
import rospy
from ieee2015_end_effector_servos.srv import EE

def open_large_servo():
    rospy.wait_for_service('open_large_servo')
    try:
        open_large_servo = rospy.ServiceProxy('open_large_servo', EE)
        resp1 = open_large_servo()
        return resp1
        print resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

#if __name__ == "__main__":
def main():
    open_large_servo()
    
