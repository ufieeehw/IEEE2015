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
    '''Rectangle Finder Class
    Implementing
    [1] http://www.zemris.fer.hr/~ssegvic/pubs/sikiric10mipro.pdf
    [2] http://stackoverflow.com/questions/6606891/

    Goal: 
    Perform affine transformation of forward-looking-camera
    image, to produce a bird's eye view

    From that bird's eye view, perform localization computations

    Note:
    Need a better way to unit test this
    '''
    @classmethod
    def execute(self, image):
        '''Return the rectangle positions in space'''
        rectangles = self.find_rectangles(image)
        for rectangle in rectangles:
            self.transform_rectangles(rectangle, image)
        return NotImplemented

    @classmethod
    def angle_between(self, v1, v2, origin=np.array([0,0])):
        '''Return angle between two vectors about a given origin'''
    
        v1_u = unit_vector(v1 - origin)
        v2_u = unit_vector(v2 - origin)
        
        dot = np.dot(v1_u, v2_u)
        angle = None
        if(dot >= 0 and dot <= 1):
            angle = np.arccos(dot)
        if angle == None:
            if (v1_u == v2_u).all():
                return 0.0
            else:
                return np.pi
        return angle
    
    @classmethod
    def find_rectangles(self, image):
        '''Detect rectangles, return edge coordinates'''
        thresh_level = 2
        final_contours = []
        dilate_kernel = np.ones((5, 5), np.uint8)
        edges = None
        contours = None
        for thresh in range(thresh_level):
            if thresh == 0:
                pass
            else:
                # Do some dynamic bullshit
                edges = image
        contours, hierarchy = cv2.findContours(np.array(edges,np.uint8),
                                               cv2.RETR_LIST, 
                                               cv2.CHAIN_APPROX_SIMPLE)
        dispimg = np.array(np.zeros((image.shape[0], image.shape[1], 3)),
                                    np.uint8)
        for contour in contours:
            arc_length = cv2.arcLength(contour, True)
            if arcLen > 0.1:
                approx = cv2.approxPolyDP(contour, arcLen * (12 / 500.0), True)
        
                conditions = (
                    len(approx) >= 4, 
                    cv2.isContourConvex(approx),
                    np.fabs(cv2.contourArea(approx)) > 30
                )
                if all(conditions):
                    sum_angle = 0
                    for j in range(2,len(approx)):
                        angle = self.angle_between(approx[j % len(approx)][0],
                                              approx[j - 2][0], 
                                              approx[j - 1][0])
                        sum_angle += angle
                    if sum_angle > 3 or sum_angle < 4:
                        final_contours.append(approx)

        # Get the corners of the rectangles as touples of touples
        return NotImplemented

    @classmethod
    def transform_rectangles(self, rectangle, image):
        '''Do the affine transformation (Matt, implement)'''

        # Given the corners of rectangles of known size (??), take those sections 
        # from the image and perform the necessary affine
        # Return affine, image
        return NotImplemented


if __name__ == '__main__':
    pass