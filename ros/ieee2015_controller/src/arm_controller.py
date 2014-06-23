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
from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion

def normalize_angle(ang):
    return ang % (2 * np.pi) - np.pi


class SCARA_Controller(object):
    '''APPLIES ONLY TO 2 DOF ARM, PROOF OF CONCEPT'''
    def __init__(self):
        rospy.init_node('SCARA_controller')
        self.length_1 = self.length_2 = 100
        self.base = np.array([0, 0], np.float32)

        self.pose_sub = rospy.Subscriber('arm_des_pose', PoseStamped, self.got_des_pose)

        self.elbow_pub = rospy.Publisher('arm_elbow_angle', Float32)
        self.base_pub = rospy.Publisher('arm_base_angle', Float32)

        self.des_position = (50, 50)
        solutions = self.solve_angles(self.des_position)
        print solutions
        self.publish_angles(*solutions)

    def got_des_pose(self, msg):
        self.des_position = np.array([msg.pose.position.x, msg.pose.position.y])
        base, elbow = self.solve_angles(self.des_position)
        self.publish_angles(base, elbow)

    def publish_angles(self, base, elbow):
        self.base_pub.publish(Float32(data=base))
        self.elbow_pub.publish(Float32(data=elbow))
        print("Publishing angles")

    def solve_angles(self, pt):
        '''2DOF has a closed form solution, this method only slightly extends to higher DOF, where there is not a closed form
        Returns None if no solution exists.
        You should *NEVER* get the numpy "Invalid input in arccos" or a NaN output - if you do, something went wrong.'''
        x, y = pt

        distance = np.sqrt((x**2) + (y**2))
        if distance > self.length_1 + self.length_2:
            return None

        mantissa = distance / (2 * self.length_1)
        base_angle = np.arctan(y / x) - np.arccos(mantissa)

        abs_joint_angle = np.arctan( (y - (self.length_1 * np.sin(base_angle))) / (x - (self.length_1 * np.cos(base_angle))) )
        joint_angle = abs_joint_angle - base_angle

        return map(normalize_angle, (base_angle, joint_angle))



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
    #for test in tests:
    #    print SCARA.solve_angles(test)

