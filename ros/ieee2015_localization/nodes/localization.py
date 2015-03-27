#!/usr/bin/env python
import cv2
import rospy
import numpy as np

from geometry_msgs.msg import Twist, Point, PoseStamped, Pose2D, Quaternion

import roslib
roslib.load_manifest('ieee2015_vision')
from ros_image_tools import Image_Subscriber
roslib.load_manifest('ieee2015_localization')
from slam.registration import similarity, translation, similarity_fast
from time import time

class Localization(object):
    def __init__(self):
        rospy.init_node('localization')

        self.map_size = 1000
        self.full_map = np.zeros((self.map_size, self.map_size), np.uint8)

        self.image_sub = Image_Subscriber('camera', self.image_cb)
        self.image = None

        self.pose_pub = rospy.Publisher('SLAM_pose', Pose2D, queue_size=3)

        self.current_position = (500, 500)


    def stitch(self, image, angle):
        rows, cols = image.shape
        rot_M = cv2.getRotationMatrix2D((cols / 2, rows / 2), angle, 1)

        rotated = cv2.warpAffine(image, rot_M, (cols, rows))
        [tx, ty] = translation(self.image, rotated[:125, :125])
        trans_M = np.float32([
            [1.0, 0.0, ty],
            [0.0, 1.0, tx]
        ])

        print '\ttranslate_1: {}, translate_2: {}'.format(tx, ty)
        translated = cv2.warpAffine(rotated, trans_M, (cols, rows))
        cv2.imshow('Destination', translated)

        self.full_map[
            self.current_position[0] + tx: self.current_position[0] + tx + 125, 
            self.current_position[1] + ty: self.current_position[1] + ty + 125
            ] = rotated

        cv2.imshow('Map', self.full_map)
        cv2.waitKey(1)


    def fix_size(self, image, size=200):
        '''Takes an image, makes it square, resizes it
        Size is an integer, and we will reshape to (size, size)'''
        shape = image.shape
        square_shape = min(shape)

        squared = image[:square_shape, :square_shape]
        sized = cv2.resize(squared, (size, size))
        return sized

    def image_cb(self, image):
        image = self.fix_size(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), size=125)
        if self.image is None:
            self.image = image
            return

        cv2.imshow('Image!', self.image)

        tic = time()
        scale, angle = similarity_fast(self.image, image)

        print 'Changes; scale: {}, angle: {}'.format(scale, angle)

        self.stitch(image, angle)
        toc = time() - tic
        print 'Single loop took {} seconds'.format(toc)


if __name__ == '__main__':
    localization = Localization()
    rospy.spin()