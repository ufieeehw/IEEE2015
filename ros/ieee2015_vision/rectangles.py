#!/usr/bin/python
from __future__ import division
## Math
import numpy as np
## Ros
import rospy
import tf.transformations as tf_trans
import tf
## Ros Msgs
from std_msgs.msg import Header, Float32
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Pose, Quaternion
## Computer Vision
import cv2

class RectangleFinder(object):

    @classmethod
    def execute(self, image):
        rectangles = self.find_rectangles(image)
        for rectangle in rectangles:
            self.transform_rectangles(rectangle, image)

        return NotImplemented

    @classmethod
    def find_rectangles(self, image):
        thresh_level = 2
        finalContours = []
        dilateKernel = np.ones((5,5),np.uint8)
        edges = None
        contours = None
        for l in range(thresh_level):
            if l == 0:
                pass
            else:
                edges = image
        contours,hierarchy = cv2.findContours(np.array(edges,np.uint8), 
                                              cv2.RETR_LIST, 
                                              cv2.CHAIN_APPROX_SIMPLE
                                             ) 
        dispimg = np.array(np.zeros(
                                    (image.shape[0], image.shape[1], 3)
                                    ), 
                                    np.uint8
                           )
        for ctr in contours:
            arcLen = cv2.arcLength(ctr,True)
            if arcLen > 0.1:
                approx = cv2.approxPolyDP(ctr,arcLen*(12/500.0), True)    
        
                if len(approx)>=4 and cv2.isContourConvex(approx) && np.fabs(cv2.contourArea(approx)) > 30
                    sumAngle = 0
                    for j in range(2,len(approx)):
                        angle = angle_between(approx[j%len(approx)][0], approx[j-2][0], approx[j-1][0])
                        sumAngle += angle
                    if sumAngle > 3 or sumAngle < 4:
                        finalContours.append(approx)

        # Get the corners of the rectangles as touples of touples
        return NotImplemented

    @classmethod
    def transform_rectangles(self, rectangle, image):

        # Given the corners of rectangles of known size (??), take those sections 
        # from the image and perform the necessary affine
        # Return affine, image
        return NotImplemented