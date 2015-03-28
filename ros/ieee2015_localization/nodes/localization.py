#!/usr/bin/env python
import cv2
import rospy
import numpy as np

import roslib
roslib.load_manifest('ieee2015_vision')
from ros_image_tools import Image_Subscriber
roslib.load_manifest('ieee2015_localization')
from slam.registration import similarity

class Localization(object):
    def __init__(self):
        rospy.init_node('localization')

        self.map_size = 2000
        self.full_map = np.zeros((self.map_size, self.map_size), np.uint8)

        self.image_sub = Image_Subscriber('camera', self.image_cb)
        self.image = None

    def stitch(self, image, fx, fy, angle):
        angle = np.radians(angle)
        c, s = np.cos(angle), np.sin(angle)

        trans_M = np.float32([
            [0.0, 0.0, fy],
            [0.0, 0.0, fx]
        ])

        rows, cols = image.shape
        rot_M = cv2.getRotationMatrix2D((cols / 2, rows / 2), np.degrees(angle), 1)

        rotated = cv2.warpAffine(image, rot_M, (cols, rows))
        translated = cv2.warpAffine(rotated, trans_M, (cols, rows))

        cv2.imshow('Destination', translated)
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
        matched, scale, angle, (t0, t1) = similarity(self.image, image)
        print 'Changes; scale: {}, angle: {}, t0: {}, t1: {}'.format(scale, angle, t0, t1)
        # cv2.imshow('Other!', matched)
        # cv2.waitKey(1)

        self.stitch(image, t0, t1, angle)


if __name__ == '__main__':
    localization = Localization()
    rospy.spin()