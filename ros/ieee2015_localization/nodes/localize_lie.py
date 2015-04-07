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
            self.image_cb, 
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
        # assert matrix.shape == (3, 3), "Requires 3x3 matrix in homogeneous space"
        a = matrix[0, 0]
        b = matrix[0, 1]
        c = matrix[1, 0]
        d = matrix[1, 1]

        scale_x = np.sign(a) * np.sqrt((a**2) + (b**2))
        scale_y = np.sign(d) * np.sqrt((c**2) + (d**2))
        angle = np.arctan2(-b, a)
        translation_x = matrix[0, 2]
        translation_y = matrix[1, 2]
        return angle, translation_x, translation_y, scale_x, scale_y

    def warp(self, image, matrix):
        return cv2.warpAffine(image, matrix, image.shape)

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

    def publish_pose(self, tx, ty, angle):
        self.pose_pub.publish(
            Pose2D(
                x=tx,
                y=ty,
                theta=angle,
            )
        )

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
        # scale, delta_angle = similarity_fast(self.keyframe_image, new_image)
        matrix = cv2.estimateRigidTransform(new_image, self.keyframe_image, False)

        warped = self.warp(new_image, matrix)
        cv2.imshow("Rewarped", warped)

        ang, dx, dy, sx, sy = self.motion_from_matrix(matrix)
        print 'Angle: {}, dx: {}, dy: {}\n\t sx: {} sy: {}'.format(ang, dx, dy, sx, sy)
        rotated = self.rotate(new_image, ang)
        self.publish_pose(dx, dy, ang)

        cv2.imshow("Rotated", rotated)

        self.overlay(
            warped, 
            self.keyframe_position[0] + 0.0,
            self.keyframe_position[1] + 0.0,
        )

        cv2.imshow('Map', self.full_map)

        cv2.waitKey(1)
        return dx, dy, ang

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
        '''Image callback, tolerates an image message

        1. Get an image, if it is the first, make it a keyframe
        2. Keyframes always have a black background
        3. Newframes always have a white background

        '''

        image_msg = np.squeeze(image_msg)
        image_fixed = self.fix_size(image_msg, size=self.shrink_size)
        ret, image = cv2.threshold(image_fixed, 200, 255, cv2.THRESH_BINARY)

        if self.keyframe_image is None:
            self.keyframe_image = image
            return


        tic = time()
        cv2.imshow("Input Image", image)
        dx, dy, ang = self.stitch(image)
        cv2.imshow("Keyframe", self.keyframe_image)

        '''
        if ((np.linalg.norm([dx, dy]) > 10) or
            (np.fabs(ang - self.keyframe_orientation) > 20)):
            # Keyframe
            self.keyframe_image = image
            # Absolute Position
            self.keyframe_position = (self.keyframe_position[0] + dx, self.keyframe_position[1] + dy)
            # Absolute ang
            self.keyframe_orientation = ang
            print 'keyframe orientation {}'.format(ang)
        '''


        cv2.waitKey(1)
        toc = time() - tic
        # print '--------Single loop took {}    --------'.format(toc)


if __name__ == '__main__':
    localization = Localization()
    rospy.spin()