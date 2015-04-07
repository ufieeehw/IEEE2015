#!/usr/bin/env python
from __future__ import division
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
from matplotlib import pyplot

class Localization(object):
    def __init__(self):
        rospy.init_node('localization')

        self.map_size = 750
        self.reset()

        self.image_sub = Image_Subscriber(
            '/robot/base_camera/down_view', 
            # self.image_cb, 
            self.thresh_cb
            encoding="8UC1",
        )

        self.pose_pub = rospy.Publisher('SLAM_pose', Pose2D, queue_size=3)

        # Size we shrink incoming images to (k x k)
        self.shrink_size = 125
        self.ones_mask = np.ones((self.shrink_size, self.shrink_size))

    def reset(self):
        self.full_map = np.zeros((self.map_size, self.map_size), np.uint8)
        self.keyframe_image = None
        self.keyframe_position = 0
        self.keyframe_scale = 1.0
        self.keyframe_orientation = 0.0
        self.keyframe_position = (self.map_size // 2, self.map_size //2)

    def reset_map(self):
        self.full_map = np.zeros((self.map_size, self.map_size), np.uint8)

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

    def overlay(self, image, x_offset, y_offset, mask=None):
        if mask is None:
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

        else:
            greater = (image > 1) & (mask < 1)
            # cv2.imshow("SHOW", (np.uint8(greater)) * 255)

            self.full_map[
                x_offset: x_offset + image.shape[0],
                y_offset: y_offset + image.shape[1],

            ] *= np.ones(image.shape) - (greater * 0.5)

            image[mask > 1] = 0
            self.full_map[
                x_offset: x_offset + image.shape[0],
                y_offset: y_offset + image.shape[1],
            ] += image * 0.5

    def stitch(self, new_image, mask=None):
        scale, delta_angle = similarity_fast(self.keyframe_image, new_image)
        absolute_angle = self.keyframe_orientation + delta_angle
        absolute_scale = scale * self.keyframe_scale

        simply_scaled = self.rotate(new_image, absolute_angle, 1.0 / scale)

        print 'Scale {}, keyframe scale {}'.format(scale, self.keyframe_scale)
        rotated_new = self.rotate(new_image, absolute_angle, 1.0 / absolute_scale)
        rotated_keyframe = self.rotate(self.keyframe_image, self.keyframe_orientation, 1.0 / self.keyframe_scale)

        [delta_x, delta_y] = translation(rotated_keyframe, rotated_new)

        # translated = self.translate(rotated_new, delta_x, delta_y)
        # cv2.imshow("Simple Scale", simply_scaled)
        # cv2.imshow("New to Abs", rotated_new)
        # cv2.imshow("Rkeyframe Image", rotated_keyframe)

        self.overlay(rotated_new, 
            self.keyframe_position[0] + delta_x,
            self.keyframe_position[1] + delta_y,
            mask
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


    def thresh_stitch(self, new_image, mask=None):
        scale, delta_angle = similarity_fast(self.keyframe_image, new_image)
        absolute_angle = self.keyframe_orientation + delta_angle
        absolute_scale = scale * self.keyframe_scale

        simply_scaled = self.rotate(new_image, absolute_angle)

        print 'Scale {}, keyframe scale {}'.format(scale, self.keyframe_scale)
        rotated_new = self.rotate(new_image, absolute_angle)
        rotated_keyframe = self.rotate(self.keyframe_image, self.keyframe_orientation)

        [delta_x, delta_y] = translation(rotated_keyframe, rotated_new)

        # translated = self.translate(rotated_new, delta_x, delta_y)
        cv2.imshow("New to Abs", rotated_new)
        cv2.imshow("Rkeyframe Image", rotated_keyframe)

        self.overlay(rotated_new, 
            self.keyframe_position[0] + delta_x,
            self.keyframe_position[1] + delta_y,
            mask
        )

        cv2.imshow('Map', self.full_map)

        return delta_x, delta_y, absolute_angle, scale

    def thresh_cb(self, image_msg):
        '''Maps using only white lines'''
        image_msg = np.squeeze(image_msg)
        image = self.fix_size(image_msg, size=self.shrink_size)

        ret, lines = cv2.threshold(image, 200, 255, cv2.THRESH_BINARY)
        if self.keyframe_image is None:
            self.keyframe_image = lines

        # ret, down_view_mask = cv2.threshold(self.keyframe_image, 10, 255, cv2.THRESH_BINARY_INV)
        ret, lines = cv2.threshold(image, 200, 255, cv2.THRESH_BINARY)

        # lines -= np.uint8(down_view_mask) * 255
        cv2.imshow("binary", lines)

        tx, ty, angle, scale = self.thresh_stitch(lines)
        print tx, ty, angle
        if ((np.linalg.norm([tx, ty]) > 10) or
            (np.fabs(angle - self.keyframe_orientation) > 20)):
            # Keyframe
            self.keyframe_image = lines
            # Absolute Position
            self.keyframe_position = (self.keyframe_position[0] + tx, self.keyframe_position[1] + ty)
            # Absolute Angle
            self.keyframe_orientation = angle
            print 'keyframe orientation {}'.format(angle)
            
        key_press = cv2.waitKey(1)
        if key_press & 0xFF == ord('r'):
            self.reset()
        if key_press & 0xFF == ord('f'):
            self.reset_map()



    def image_cb(self, image_msg):
        '''Image callback, tolerates an image message

        1. Get an image, if it is the first, make it a keyframe
        2. Keyframes always have a black background
        3. Newframes always have a white background

        '''

        image_msg = np.squeeze(image_msg)
        image = self.fix_size(image_msg, size=self.shrink_size)

        if self.keyframe_image is None:
            self.keyframe_image = image
            return

        ret, down_view_mask = cv2.threshold(self.keyframe_image, 10, 255, cv2.THRESH_BINARY_INV)

        image -= np.uint8(down_view_mask) * 255
        cv2.imshow("binary", image)

        tic = time()
        tx, ty, angle, scale = self.stitch(image, down_view_mask)

        if ((np.linalg.norm([tx, ty]) > 10) or
            (np.fabs(angle - self.keyframe_orientation) > 20) or 
            (scale > 1.3) or (scale < 0.75)):
            # Keyframe
            image += np.uint8(down_view_mask) * 255
            self.keyframe_image = image
            # Absolute Position
            self.keyframe_position = (self.keyframe_position[0] + tx, self.keyframe_position[1] + ty)
            # Absolute Angle
            self.keyframe_orientation = angle
            print 'keyframe orientation {}'.format(angle)
            # Absolute Scale
            print 'Changing scale'
            self.keyframe_scale = self.keyframe_scale * scale

        toc = time() - tic
        print '--------Single loop took {}    --------'.format(toc)


if __name__ == '__main__':
    localization = Localization()
    rospy.spin()