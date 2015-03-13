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
    def __init__(self, lengths=(0.148, 0.160)):
        rospy.init_node('SCARA_controller')
        self.shoulder_length, self.elbow_length = lengths
        self.base = np.array([0.0, 0.0], np.float32)

        self.pose_sub = rospy.Subscriber('arm_des_pose', PointStamped, self.got_des_pose, queue_size=1)

        self.elbow_pub = rospy.Publisher('elbow_controller/command', Float64, queue_size=1)
        self.shoulder_pub = rospy.Publisher('shoulder_controller/command', Float64, queue_size=1)
        self.des_position = None

    def got_des_pose(self, msg):
        '''Receiving desired pose'''
        self.des_position = np.array([msg.point.x, msg.point.y])
        solutions = self.solve_angles(self.des_position)
        if solutions is not None:
            print("Targeting arm-shoulder local position {} (m)".format(self.des_position))
            shoulder, elbow = solutions
            self.publish_angles(shoulder, elbow)
        else:
            print "Could not find an arm movement solution"

    def publish_angles(self, shoulder, elbow):
        '''Publish shoulder and elbow angles
        TODO:
            Add shoulder angle
        '''
        print "Targeting angles shoulder: {0:0.2f} (rad), elbow: {0:0.2f} (rad)".format(shoulder, elbow)
        shoulder_angle_offset = 0.3 - np.pi/2
        # elbow_angle_offset = 1.75
        elbow_angle_offset = np.pi / 6

        _shoulder_angle = shoulder + shoulder_angle_offset
        _elbow_angle = np.pi - (elbow + elbow_angle_offset)

        self.shoulder_pub.publish(Float64(data=_shoulder_angle))
        self.elbow_pub.publish(Float64(data=_elbow_angle))

    def solve_angles(self, pt):
        '''2DOF has a closed form solution, this method only slightly extends to higher DOF, where there is not a closed form
        Returns None if no solution exists.
        You should *NEVER* get the numpy "Invalid input in arccos" or a NaN output - if you do, something went wrong.'''
        x, y = pt

        distance = np.sqrt((x**2) + (y**2))
        if distance > (self.shoulder_length + self.elbow_length):
            print("Target arm position out of bounds")
            return None
        elif(y > 0.160):
            print('Arm reaching too high - some links will collide near the base');
            return None
        elif(y < -0.105):
            print('Arm reaching too low - possible collision with the ground');
            return None
        elif(x < 0.025):
            print('Arm is going to collide with itself');
            return None
        elif(y < 0.00 and x < 0.185):
            print('Arm is going to collide with the chassis');
            return None
        elif(y > 0.080 and x < 0.050):
            print('Arm is going to collide with its back-carriage');
            return None

        # SCARA (Elbow-Angle wrt shoulder) implementation
        '''
        shoulder_angle = np.arctan2(y, x) - np.arccos(distance / (2 * self.shoulder_length))

        abs_joint_angle = np.arctan2(y - (self.shoulder_length * np.sin(shoulder_angle)),
                                     x - (self.shoulder_length * np.cos(shoulder_angle)))
        # joint_angle = abs_joint_angle - shoulder_angle
        elbow_angle = abs_joint_angle
        '''
        # u-arm (Elbow-Angle wrt ground)
        c2 = ((self.elbow_length**2 + self.shoulder_length**2) - (x**2 + y**2))/(2 * self.elbow_length * self.shoulder_length)
        s2 = np.sqrt(1 - c2**2)

        c1 = (self.shoulder_length * x - self.elbow_length * (x * c2 + y * s2))/(self.shoulder_length**2 + ((self.elbow_length * s2)**2 + (self.elbow_length * c2)**2) - 2 * self.shoulder_length * self.elbow_length * c2)
        shoulder_angle = np.arccos(c1)
        elbow_angle = shoulder_angle + np.arccos(c2)

        if (abs(x - (self.shoulder_length * np.arccos(shoulder_angle) - self.elbow_length * np.arccos(elbow_angle))) > 10**-6 or abs(y - (self.shoulder_length * np.sin(shoulder_angle) - self.elbow_length * np.sin(elbow_angle)) > 10**-6)):
            shoulder_angle =  - shoulder_angle
            elbow_angle = shoulder_angle + np.arccos(c2)

        return (shoulder_angle, elbow_angle)

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