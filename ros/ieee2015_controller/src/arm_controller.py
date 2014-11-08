#!/usr/bin/python
from __future__ import division
## Math
import numpy as np
## Ros
import rospy
import tf.transformations as tf_trans
import tf
## Ros Msgs
from std_msgs.msg import Header, Float32, Float64
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Pose, Quaternion


def normalize_angle(ang):
    '''Constrain angle between -pi and pi'''
    return ang % (2 * np.pi) - np.pi


class SCARA_Controller(object):
    '''APPLIES ONLY TO 2 DOF ARM, PROOF OF CONCEPT'''
    def __init__(self, lengths=(148, 160)):
        rospy.init_node('SCARA_controller')
        self.length_1, self.length_2 = lengths
        self.base = np.array([0, 0], np.float32)

        self.pose_sub = rospy.Subscriber('arm_des_pose', PointStamped, self.got_des_pose)

        self.elbow_pub = rospy.Publisher('arm_elbow_angle', Float32, queue_size=1)
        self.elbow_pub2 = rospy.Publisher('robot/joint3_position_controller/command', Float64, queue_size=2)
        self.base_pub = rospy.Publisher('arm_base_angle', Float32, queue_size=1)
        self.base_pub2 = rospy.Publisher('robot/joint2_position_controller/command', Float64, queue_size=2)
        self.des_position = None

    def got_des_pose(self, msg):
        '''Receiving desired pose'''
        self.des_position = np.array([msg.point.x, msg.point.y])
        solutions = self.solve_angles(self.des_position)
        if solutions is not None:
            print("Targeting arm-base local position {} (m)".format(self.des_position))
            base, elbow = solutions
            self.publish_angles(base, elbow)

    def publish_angles(self, base, elbow):
        '''This should be goddamn obvious'''
        print("Targeting angles base: {0:0.2f} (rad), elbow: {0:0.2f} (rad)".format(base, elbow))
        print(base)
        print(elbow)

        if base >= 0:
            base2 = np.pi - base
        else:
            base2 = -np.pi - base

        '''if elbow >= 0:
            elbow2 = np.pi - elbow
        else:
            elbow2 = np.pi - elbow'''
        elbow2 = elbow

        self.base_pub.publish(Float32(data=base))
        self.base_pub2.publish(Float64(data=-base2))
        self.elbow_pub.publish(Float32(data=elbow))
        self.elbow_pub2.publish(Float64(data=elbow2))

    def solve_angles(self, pt):
        '''2DOF has a closed form solution, this method only slightly extends to higher DOF, where there is not a closed form
        Returns None if no solution exists.
        You should *NEVER* get the numpy "Invalid input in arccos" or a NaN output - if you do, something went wrong.'''
        x, y = pt

        distance = np.sqrt((x**2) + (y**2))
        if distance > (self.length_1 + self.length_2):
            print("Target arm position out of bounds")
            return None

        base_angle = np.arctan2(y, x) - np.arccos(distance / (2 * self.length_1))


        if base_angle < -1.57:
            base_angle = base_angle + 2*np.pi
            if base_angle > 3.14:
                base_angle = 3.14
        if base_angle > -1.57 and base_angle < 0:
                base_angle = 0
        abs_joint_angle = np.arctan2(y - (self.length_1 * np.sin(base_angle)),
                                     x - (self.length_1 * np.cos(base_angle)))
        joint_angle = abs_joint_angle - base_angle

        '''return (normalize_angle(base_angle + np.pi), normalize_angle(np.pi + joint_angle))'''
        return (base_angle, normalize_angle(np.pi + joint_angle))

if __name__ == '__main__':
    tests = map(
        lambda o: np.array(o, np.float32),
        [
            [0.5, 0.5],
            [21, 21],
            [500, 500],
            [12, 13],
            [4, 9],
        ]
    )

    SCARA = SCARA_Controller()
    rospy.spin()