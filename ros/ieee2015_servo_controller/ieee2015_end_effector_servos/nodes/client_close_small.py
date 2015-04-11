#!/usr/bin/env python

import sys
import rospy
from ieee2015_end_effector_servos.srv import EE

def close_small_servo():
    rospy.wait_for_service('close_small_servo')
    try:
        mode_small_close = rospy.ServiceProxy('close_small_servo', EE)
        resp1 = mode_small_close()
        return resp1
        print resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

#if __name__ == "__main__":
def main():
    close_small_servo()
    
