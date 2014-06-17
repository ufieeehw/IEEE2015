#!/usr/bin/python
## Math
import numpy as np
## Ros
import rospy
import tf.transformations as tf_trans
import tf
## Ros Msgs

class SCARA_Controller(object):
    def __init__(self):
        rospy.init_node('SCARA_controller')

