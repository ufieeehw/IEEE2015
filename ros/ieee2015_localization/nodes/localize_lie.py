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
from collections import deque

'''To use this, do:
rosrun ieee2015_simulator view_simulation
rosrun ieee2015_localization locaalize_lie.py
'''



class Keyframe(object):
    def __init__(self):
        pass

class Localization(object):
    def __init__(self):
        rospy.init_node('localization')

        self.map_size = 750
        self.reset()

        self.image_sub = Image_Subscriber(
            '/robot/base_camera/down_view', 
            self.image_cb_matrix,
            encoding="8UC1",
        )

        self.pose_pub = rospy.Publisher('SLAM_pose', Pose2D, queue_size=3)

        # Size we shrink incoming images to (k x k)
        self.shrink_size = 200
        self.ones_mask = np.ones((self.shrink_size, self.shrink_size))

    def reset(self):
        self.full_map = np.zeros((self.map_size, self.map_size), np.uint8)
        self.keyframe_image = None
        self.keyframe_position = 0
        self.keyframe_scale = 1.0
        self.keyframe_orientation = 0.0
        self.keyframe_position = (self.map_size // 2, self.map_size //2)

        self.h_k2root = np.eye(3)

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
        angle = -np.arctan2(-b, a)
        translation_x = matrix[0, 2]
        translation_y = matrix[1, 2]
        return angle, translation_x, translation_y, scale_x, scale_y

    def warp(self, image, matrix):
        matrix = matrix[:2, :]
        return cv2.warpAffine(image, matrix, image.shape)

    def make_homogeneous(self, matrix):
        assert max(matrix.shape) == 3, "Matrix must have 3 columns!"
        return np.vstack([matrix, [0, 0, 1]])

    def make_2D_rotation(self, angle):
        c, s = np.cos(angle), np.sin(angle)
        mat = np.matrix([
            [c,     -s],
            [s,      c],
        ],
        dtype=np.float32)
        return mat

    def rotate(self, image, angle, scale=1.0):
        rows, cols = image.shape
        # rot_M = cv2.getRotationMatrix2D((cols / 2, rows / 2), angle, scale)
        rot_M = cv2.getRotationMatrix2D((0.0 / 2, 0.0 / 2), angle, scale)
        rotated = cv2.warpAffine(image, rot_M, (cols, rows))
        return rotated

    def translate(self, image, tx, ty, scale=1.0):
        rows, cols = image.shape
        trans_M = np.float32([
            [1.0 / scale, 0.0, tx],
            [0.0, 1.0 / scale, ty],
        ])
        translated = cv2.warpAffine(image, trans_M, (cols, rows))
        return translated

    def publish_pose(self, tx, ty, angle):
        self.pose_pub.publish(
            Pose2D(
                x=tx,
                y=ty,
                theta=angle,
            )
        )

    def overlay(self, image, x_offset, y_offset, other_image):
        greater = image > 1
        # Image Blending
        other_image[
            x_offset: x_offset + image.shape[0],
            y_offset: y_offset + image.shape[1],
        ] *= np.ones(image.shape) - (greater * 0.5)

        other_image[
            x_offset: x_offset + image.shape[0],
            y_offset: y_offset + image.shape[1],
        ] += image * 0.5

    def stitch_matrix(self, new_image):
        '''stitch_matrix(new_image)
            h -> homogeneous
            i -> image
            k -> keyframe
            root -> root
        '''
        im_to_keyframe = cv2.estimateRigidTransform(new_image, self.keyframe_image, False) 
        h_im_to_keyframe = self.make_homogeneous(im_to_keyframe)
        ang_i2k, dx_i2k, dy_i2k, sx, sy = self.motion_from_matrix(im_to_keyframe)

        h_i2root = np.dot(self.h_k2root, h_im_to_keyframe)

        ang_i2root, dx_i2root, dy_i2root, sx, sy = self.motion_from_matrix(h_i2root)

        rotated = self.rotate(new_image, np.degrees(ang_i2root))
        cv2.imshow("Rotated", rotated)

        self.overlay(
            rotated, 
            self.keyframe_position[0] + dy_i2root,
            self.keyframe_position[1] + dx_i2root,
            self.full_map
        )

        cv2.imshow("Map", self.full_map)

        return h_im_to_keyframe

    def image_cb_matrix(self, image_msg):
        key_press = cv2.waitKey(1)
        if key_press & 0xFF == ord('r'):
            self.reset()
        if key_press & 0xFF == ord('f'):
            self.reset_map()

        image_msg = np.squeeze(image_msg)
        image_fixed = self.fix_size(image_msg, size=self.shrink_size)
        ret, image = cv2.threshold(image_fixed, 200, 255, cv2.THRESH_BINARY)
        # image = image_fixed
        if self.keyframe_image is None:
            self.keyframe_image = image
            cv2.imshow("Original keyframe", image)
            return

        tic = time()
        cv2.imshow("Input Image", image)
        h_i2k = self.stitch_matrix(image)
        ang_i2k, dx_i2k, dy_i2k, sx, sy = self.motion_from_matrix(h_i2k)

        if (np.linalg.norm([dx_i2k, dy_i2k]) > 4 or
            np.fabs(ang_i2k) > 0.05):
            print "Keyframing-----"
            self.keyframe_image = image
            self.h_k2root = np.dot(self.h_k2root, h_i2k)
            # This is correct, evidence:
            cv2.imshow("warped", self.warp(image, self.h_k2root))
            print self.h_k2root

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


if __name__ == '__main__':
    localization = Localization()
    rospy.spin()