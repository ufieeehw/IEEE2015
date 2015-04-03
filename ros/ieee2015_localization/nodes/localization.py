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
import math

class Localization(object):
    def __init__(self):
        rospy.init_node('localization')

        self.map_size = 500
        self.full_map = np.zeros((self.map_size, self.map_size), np.uint8)

        self.image_sub = Image_Subscriber('/robot/base_camera/image_raw', self.image_cb)
        self.keyframe_image = None

        self.pose_pub = rospy.Publisher('SLAM_pose', Pose2D, queue_size=3)

        self.keyframe_position = (250, 250)
        self.keyframe_orientation = 0
        self.keyframe_scale = 1.0

        # Size we shrink incoming images to (k x k)
        self.shrink_size = 125
        self.ones_mask = np.ones((self.shrink_size, self.shrink_size))

    def motion_from_matrix(self, matrix):
        assert matrix.shape == (3, 3), "Requires 3x3 matrix in homogeneous space"
        sin = matrix[1, 0]
        cos = matrix[1, 1]
        angle = math.degrees(math.asin(sin))
        tx = matrix[0, 2]
        ty = matrix[1, 2]

        return angle, tx, ty

    def rotate(self, image, angle, scale=1.0):
        rows, cols = image.shape
        rot_M = cv2.getRotationMatrix2D((cols / 2, rows / 2), angle, scale)
        rotated = cv2.warpAffine(image, rot_M, (cols, rows))
        return rotated

    def translate(self, image, tx, ty, scale=1.0):
        rows, cols = image.shape
        trans_M = np.float32([
            [1.0 / scale, 0.0, ty],
            [0.0, 1.0 / scale, tx],
        ])
        translated = cv2.warpAffine(image, trans_M, (cols + ty, rows + tx))
        return translated

    def overlay(self, image, x_offset, y_offset):
        greater = image > 1
        # Image Blending
        self.full_map[
            x_offset: x_offset + image.shape[0],
            y_offset: y_offset + image.shape[1],
        ] *= np.ones(image.shape) - (greater * 0.5)

        self.full_map[
            x_offset: x_offset + image.shape[0],
            y_offset: y_offset + image.shape[1],
        ] += image * 0.5

    def stitch_2(self, new_image):
        scale, delta_angle = similarity_fast(self.keyframe_image, new_image)
        absolute_angle = self.keyframe_orientation + delta_angle
        absolute_scale = scale * self.keyframe_scale

        simply_scaled = self.rotate(new_image, absolute_angle, 1.0 / scale)

        print 'Scale {}, seed scale {}'.format(scale, self.keyframe_scale)
        rotated_new = self.rotate(new_image, absolute_angle, 1.0 / absolute_scale)
        rotated_seed = self.rotate(self.keyframe_image, self.keyframe_orientation, 1.0 / self.keyframe_scale)

        [delta_x, delta_y] = translation(rotated_seed, rotated_new)

        # translated = self.translate(rotated_new, delta_x, delta_y)
        cv2.imshow("Simple Scale", simply_scaled)
        cv2.imshow("New to Abs", rotated_new)
        cv2.imshow("RSeed Image", rotated_seed)

        self.overlay(rotated_new, 
            self.keyframe_position[0] + delta_x,
            self.keyframe_position[1] + delta_y,
        )

        cv2.imshow('Map', self.full_map)

        cv2.waitKey(1)
        return delta_x, delta_y, absolute_angle, scale

    def fix_size(self, image, size=200):
        '''Takes an image, makes it square, resizes it
        Size is an integer, and we will reshape to (size, size)'''
        shape = image.shape
        square_shape = min(shape)
        half_shape = square_shape // 2

        center_x, center_y = shape[0] // 2, shape[1] // 2

        squared = image[center_x - half_shape:center_x + half_shape, center_y - half_shape:center_y + half_shape]
        sized = cv2.resize(squared, (size, size))
        return sized

    def image_cb(self, image_msg):
        image = self.fix_size(cv2.cvtColor(image_msg, cv2.COLOR_BGR2GRAY), size=self.shrink_size)
        if self.keyframe_image is None:
            self.keyframe_image = image
            return

        cv2.imshow('keyframe_Img', self.keyframe_image)

        tic = time()
        tx, ty, angle, scale = self.stitch_2(image)

        if ((np.linalg.norm([tx, ty]) > 10) or
            (np.fabs(angle - self.keyframe_orientation) > 20) or 
            (scale > 1.3) or (scale < 0.75)):
            # Keyframe
            self.keyframe_image = image
            # Absolute Position
            self.keyframe_position = (self.keyframe_position[0] + tx, self.keyframe_position[1] + ty)
            # Absolute Angle
            self.keyframe_orientation = angle
            print 'Seed orientation {}'.format(angle)
            # Absolute Scale
            print 'Changing scale'
            self.keyframe_scale = self.keyframe_scale * scale

        toc = time() - tic
        # print '--------Single loop took {}    --------'.format(toc)


if __name__ == '__main__':
    localization = Localization()
    rospy.spin()