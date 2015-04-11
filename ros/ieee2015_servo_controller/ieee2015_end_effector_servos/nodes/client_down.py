#!/usr/bin/env python

import sys
import rospy
from ieee2015_end_effector_servos.srv import EE

def mode_down_client():
    rospy.wait_for_service('mode_down')
    try:
        mode_DOWN = rospy.ServiceProxy('mode_down', EE)
        resp1 = mode_DOWN()
        return resp1
        print resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

#if __name__ == "__main__":
def main():
    mode_down_client()
    
