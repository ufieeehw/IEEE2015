#!/usr/bin/env python
import rospy
import pygame
import random
import os, sys

from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Point, PoseStamped, Pose, Quaternion
from tf import transformations as tf_trans

import math
import numpy as np

    def publish_pose(self):
        '''Publish Pose
        '''
        _orientation = tf_trans.quaternion_from_euler(0,0,self.rad_angle)
        self.pose_pub.publish(
            PoseStamped(
                header = Header(
                    stamp=rospy.Time.now(),
                    frame_id='/course',
                ),
                pose = Pose(
                    position = Point(self.position_vector[0], self.position_vector[1], 0.0),
                    orientation = Quaternion(*_orientation), #Radians
                )
            )
        )

