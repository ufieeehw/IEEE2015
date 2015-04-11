#!/usr/bin/env python

import sys
import rospy
from ieee2015_end_effector_servos.srv import EE

def open_small_servo():
    rospy.wait_for_service('open_small_servo')
    try:
        open_small_servo = rospy.ServiceProxy('open_small_servo', EE)
        resp1 = open_small_servo()
        return resp1
        print resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

#if __name__ == "__main__":
def main():
    open_small_servo()
    
