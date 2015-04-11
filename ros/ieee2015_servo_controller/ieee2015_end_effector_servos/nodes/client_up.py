#!/usr/bin/env python

import sys
import rospy
from ieee2015_end_effector_servos.srv import EE

def mode_up_client():
    rospy.wait_for_service('mode_up')
    try:
        mode_UP = rospy.ServiceProxy('mode_up', EE)
        resp1 = mode_UP()
        return resp1
        print resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

#if __name__ == "__main__":
def main():
    mode_up_client()
    
