#!/usr/bin/env python
from __future__ import division
import rospy
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped, Vector3, Point, Quaternion
from ieee2015_msgs.msg import Mecanum
from ieee2015_msgs.srv import StopController, StopControllerResponse, ResetOdom, ResetOdomResponse
from std_msgs.msg import Header
from nav_msgs.msg import Odometry

import roslib
roslib.load_manifest('ieee2015_vision')
from object_detection.find_obj_square import find_obj_square, find_obj_square_2
from ros_image_tools import Image_Subscriber
roslib.load_manifest('ieee2015_localization')
from slam.utils.detect_squares import nav_detect_squares
from slam.utils.scaling import cv_to_robot
import numpy as np
import tf
import cv2


def xyzw_array(quaternion):
    '''Convert quaternion to non-shit array'''
    xyzw_array = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    return(xyzw_array)



class Compete(object):
    '''Mission Control
    Compete:

    Jacob's Goddamn Plan:
        1. Start; detect squares
        2. Gently rotate and continue to detect squares
        3. Place those squares in the appropriate frame
        4. Once we have found 4 (or timeout), target the bottom left one (min x, then min y)
        5. Drive there, then orient correctly
        6. Drive to the middle of the board, then drive again to the next target

    TODO: 
        - Convert points in image space to world space (need to do this universally somehow)
        - Add TF nonsense for converting from robot to world frame using odometry
        - Magnetometer absolute-orientation integration

    '''
    def __init__(self):
        rospy.init_node('mission_control')
        self.image = None
        self.down_view_sub = Image_Subscriber(
            '/robot/base_camera/down_view',
            self.down_view_cb,
            encoding="8UC1",
        )
        self.image_scale = 0.3 / 45.88  # m / px
        self.position = np.array([0.0, 0.0])
        self.yaw = 0.0
        self.odom_sub = rospy.Subscriber('/robot/odom', Odometry, self.got_odom)
        self.desired_pose_pub = rospy.Publisher('/robot/desired_pose', PoseStamped)
        print '---------waiting for odom reset-----'
        rospy.wait_for_service('/robot/reset_odom')
        self.reset_odom = rospy.ServiceProxy('/robot/reset_odom', ResetOdom)

    def down_view_cb(self, img_msg):
        self.image = img_msg

    # def pose_callback(self, pose_msg):

    def got_odom(self, msg):
        pose = msg.pose.pose
        self.position = np.array([pose.position.x, pose.position.y])
        self.yaw = tf.transformations.euler_from_quaternion(xyzw_array(pose.orientation))[2]

        # self.position = np.array([pose_msg.pose.position.x, pose_msg.pose.position.y])
        # self.yaw = tf.transformations.quaternion_from_euler(pose_msg.pose.orientation)[2]

    def norm_angle_diff(self, ang_1, ang_2):
        '''norm_angle_diff(ang_1, ang_2)
        -> Normalized angle difference, constrained to range [-pi, pi]
        '''
        return ((ang_1 - ang_2 + np.pi) % (2 * np.pi)) - (np.pi)

    def publish_desired_pose(self, x, y, yaw):
        _orientation = tf.transformations.quaternion_from_euler(0, 0, yaw)
        self.desired_pose_pub.publish(
            PoseStamped(
                header = Header(
                    stamp=rospy.Time.now(),
                    frame_id='/course',
                ),
                pose = Pose(
                    position = Point(x, y, 0.0),
                    orientation = Quaternion(*_orientation), #Radians
                )
            )
        )

    def goto_pose(self, (des_x, des_y), des_yaw):
        self.publish_desired_pose(des_x, des_y, des_yaw)

        while not rospy.is_shutdown():
            position_error = np.array([des_x, des_y]) - self.position
            yaw_error = self.norm_angle_diff(des_yaw, self.yaw)
            if (np.linalg.norm(position_error) < 0.07) and (np.fabs(yaw_error) < 0.03):
                break
        return

    def find_closest_square(self):
        squares = nav_detect_squares(
            self.image,
            shape=self.image.shape,
            offset=0.235,
            scale=self.image_scale,
        )
        if len(squares) == 0:
            return None
        target = min(squares, key=lambda o:np.linalg.norm(self.position - o))
        return target


    def scan(self, min_ang, max_ang):
        print 'scanning'
        best_dist = np.inf
        best_pt = None
        best_yaw = None

        des_yaw = min_ang
        self.publish_desired_pose(self.position[0], self.position[1], min_ang)
        while not rospy.is_shutdown():
            # position_error = np.array([des_x, des_y]) - self.position
            yaw_error = self.norm_angle_diff(des_yaw, self.yaw)

            closest = self.find_closest_square()
            if closest is None:
                continue
            dist = np.linalg.norm(closest - self.position)
            if dist < best_dist:
                best_dist = dist
                best_pt = closest
                best_yaw = self.yaw

            if np.linalg.norm(yaw_error) < 0.07:
                break

        des_yaw = max_ang
        self.publish_desired_pose(self.position[0], self.position[1], max_ang)
        while not rospy.is_shutdown():
            # position_error = np.array([des_x, des_y]) - self.position
            yaw_error = self.norm_angle_diff(des_yaw, self.yaw)

            closest = self.find_closest_square()
            if closest is None:
                continue
            dist = np.linalg.norm(closest - self.position)
            if dist < best_dist:
                best_dist = dist
                best_pt = closest
                best_yaw = self.yaw

            if np.linalg.norm(yaw_error) < 0.07:
                break

        return best_pt, best_yaw

    def run(self):
        # while self.image is None:
        state = 'start'
        while not rospy.is_shutdown():
            if self.image is None:
                continue

            if state == 'start':
                self.goto_pose((0.1, 0.0), 0.0)
                state = 'first_find'

            if state = 'first_find':
                # This returns robot-frame position
                best_pt, best_yaw = self.scan(-np.pi / 2., np.pi / 2.)
                print best_yaw
                exit()
                self.goto_pose((self.position[0], self.position[1]), best_yaw)
                cv2.waitKey(1)
                # self.goto_pose(target, 0.0)
                print 'Target achieved'
                self.image = None
                # break


if __name__ == '__main__':
    c = Compete()
    c.run()