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

        self.image_sub = Image_Subscriber('camera', self.image_cb)
        self.seed_image = None

        self.pose_pub = rospy.Publisher('SLAM_pose', Pose2D, queue_size=3)

        self.current_seed_position = (250, 250)
        self.current_seed_orientation = 0

        # Size we shrink incoming images to (k x k)
        self.shrink_size = 125
        self.ones_mask = np.ones((125, 125))

    def motion_from_matrix(self, matrix):
        assert matrix.shape == (3, 3), "Requires 3x3 matrix in homogeneous space"
        sin = matrix[1, 0]
        cos = matrix[1, 1]
        angle = math.degrees(math.asin(sin))
        tx = matrix[0, 2]
        ty = matrix[1, 2]

        return angle, tx, ty

    def rotate(self, image, angle):
        rows, cols = image.shape
        rot_M = cv2.getRotationMatrix2D((cols / 2, rows / 2), angle, 1)
        rotated = cv2.warpAffine(image, rot_M, (cols, rows))
        return rotated

    def translate(self, image, tx, ty, scale=1.0):
        rows, cols = image.shape
        trans_M = np.float32([
            [1.0 / scale, 0.0, ty],
            [0.0, 1.0 / scale, tx]
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
        scale, delta_angle = similarity_fast(self.seed_image, new_image)
        absolute_angle = self.current_seed_orientation + delta_angle

        rotated_new = self.rotate(new_image, absolute_angle)
        rotated_seed = self.rotate(self.seed_image, self.current_seed_orientation)

        [delta_x, delta_y] = translation(rotated_seed, rotated_new)

        # translated = self.translate(rotated_new, delta_x, delta_y)
        cv2.imshow("New_rotation", rotated_new)
        cv2.imshow("Re_rotated", rotated_seed)

        self.overlay(rotated_new, 
            self.current_seed_position[0] + delta_x,
            self.current_seed_position[1] + delta_y,
        )

        cv2.imshow('Map', self.full_map)

        cv2.waitKey(1)
        return delta_x, delta_y, absolute_angle, scale

    def stitch(self, new_image):
        '''Map stitching, stitch(new_image, angle, scale) -> tx, ty, angle, scale

        Determine the transformation from new_image to seed new_image. 
         This transformation is equivalent to the motion of the robot
        '''
        rows, cols = new_image.shape
        assert (rows, cols) == (self.shrink_size, self.shrink_size), "Images must be square and the same size"

        # Get rotation to seed image
        scale, delta_angle = similarity_fast(self.seed_image, new_image)  

        # Accomodate for the fact that we are using new seed images frequently
        true_angle = delta_angle + self.current_seed_orientation

        rotation_matrix = cv2.getRotationMatrix2D((cols / 2, rows / 2), delta_angle, 1)
        rotated = cv2.warpAffine(new_image, rotation_matrix, (cols, rows))
        cv2.imshow("Rotated", rotated)

        # Get translation to seed image
        [delta_x, delta_y] = translation(self.seed_image, rotated)

        print 'Motion; d_scale: {}, d_angle: {}'.format(scale, delta_angle)
        print '\ttranslate_1: {}, translate_2: {}'.format(delta_x, delta_y)

        print 'True angle: {}, true x: {}, true y: {}'.format(true_angle, self.current_seed_position[0] + delta_x, self.current_seed_position[1] + delta_y)

        # Rotate to absolute map orientation
        rotation_matrix_abs = cv2.getRotationMatrix2D((cols / 2, rows / 2), self.current_seed_orientation, 1)
        rotation_abs = cv2.warpAffine(rotated, rotation_matrix_abs, (cols, rows))
        cv2.imshow('Destination', rotation_abs)

        rad_current_orientation = math.radians(self.current_seed_orientation)
        c1, s1 = math.cos(rad_current_orientation), math.sin(rad_current_orientation)
        A = np.matrix([
            [+c1, -s1, +self.current_seed_position[0] - 250],
            [+s1, +c1, +self.current_seed_position[1] - 250],
            [0, 0, 1],
        ])

        radang = math.radians(delta_angle)
        c2, s2 = math.cos(radang), math.sin(radang)
        B = np.matrix([
            [+c2, -s2, -delta_x],
            [+s2, +c2, -delta_y],
            [0,     0,        1],
        ])

        # trans = np.matrix([delta_x, delta_y, 1]).T
        # noting, (A * B).I = (B.I * A.I)
        true_affine = A.I * B.I
        full_angle, full_x, full_y = self.motion_from_matrix(true_affine)

        # print true_x - delta_x, true_y - delta_y
        # true_x, true_y = delta_x, delta_y


        # Overwrite a section of the map with new match
        # self.full_map[
        #     self.current_seed_position[0] + delta_x: self.current_seed_position[0] + delta_x + self.shrink_size, 
        #     self.current_seed_position[1] + delta_y: self.current_seed_position[1] + delta_y + self.shrink_size,
        #     ] = rotation_abs
        # self.full_map[
        #     self.current_seed_position[0] + true_x: self.current_seed_position[0] + true_x + self.shrink_size, 
        #     self.current_seed_position[1] + true_y: self.current_seed_position[1] + true_y + self.shrink_size,
        # ] = rotation_abs
        self.full_map[
            full_x + 250: full_x + 250 + self.shrink_size, 
            full_y + 250: full_y + 250 + self.shrink_size,
        ] = rotation_abs


        cv2.imshow('Map', self.full_map)
        cv2.waitKey(1)
        return(delta_x, delta_y, true_angle, scale)


    def fix_size(self, image, size=200):
        '''Takes an image, makes it square, resizes it
        Size is an integer, and we will reshape to (size, size)'''
        shape = image.shape
        square_shape = min(shape)

        squared = image[:square_shape, :square_shape]
        sized = cv2.resize(squared, (size, size))
        return sized

    def image_cb(self, image_msg):
        image = self.fix_size(cv2.cvtColor(image_msg, cv2.COLOR_BGR2GRAY), size=self.shrink_size)
        if self.seed_image is None:
            self.seed_image = image
            return

        cv2.imshow('Image!', self.seed_image)

        tic = time()
        # self.stitch_2(image)
        tx, ty, angle, scale = self.stitch_2(image)

        if (np.linalg.norm([tx, ty]) > 10) or (np.fabs(angle - self.current_seed_orientation) > 20):
            self.seed_image = image
            self.current_seed_position = (self.current_seed_position[0] + tx, self.current_seed_position[1] + ty)
            self.current_seed_orientation = angle

        toc = time() - tic
        print '--------Single loop took {} seconds--------'.format(toc)


if __name__ == '__main__':
    localization = Localization()
    rospy.spin()