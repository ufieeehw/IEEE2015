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
    def __init__(self):
        rospy.init_node('SCARA_controller')
        self.shoulder_length, self.elbow_length = (0.148, 0.160)
        self.base = np.array([0.0, 0.0], np.float32)

        ## Offsets
        # Angles
        self.shoulder_angle_offset = 1.06
        self.elbow_angle_offset = 0.65
        self.base_angle_offset = 0.0
        # Distances
        self.base_arm_offset = 0.02073  # in x direction

        self.pose_sub = rospy.Subscriber('arm_des_pose', PointStamped, self.got_des_pose, queue_size=1)

        self.elbow_pub = rospy.Publisher('elbow_controller/command', Float64, queue_size=1)
        self.shoulder_pub = rospy.Publisher('shoulder_controller/command', Float64, queue_size=1)
        self.base_pub = rospy.Publisher('base_controller/command', Float64, queue_size=1)
        self.des_position = None

    def got_des_pose(self, msg):
        '''Receiving desired pose'''
        self.des_position = np.array([msg.point.x, msg.point.y, msg.point.z])
        base_solution = self.solve_base_angle(self.des_position[:2])  # x and y

        # Norm -> projected X length
        scara_x = np.linalg.norm([msg.point.x, msg.point.y]) - self.base_arm_offset
        # Up-ness of arm
        scara_y = msg.point.z
        arm_solution = self.solve_arm_angles((scara_x, scara_y))

        if (arm_solution is not None) and (base_solution is not None):
            # rospy.loginfo("Targeting arm-shoulder local position {} (m)".format(self.des_position))
            shoulder, elbow = arm_solution
            base = base_solution
            self.publish_angles(shoulder, elbow, base)
        else:
            rospy.logwarn("Could not find an arm movement solution")

    def publish_angles(self, shoulder, elbow, base):
        '''Publish shoulder and elbow angles
        TODO:
            Add shoulder angle
        '''
        # rospy.loginfo("Targeting angles shoulder: {0:0.2f} (rad), elbow: {0:0.2f} (rad)".format(shoulder, elbow))

        # Apply the inverse of the angle correction to get the servo angles from arm angles
        _shoulder_angle = shoulder - self.shoulder_angle_offset
        _elbow_angle = np.pi -(elbow + self.elbow_angle_offset)
        _base_angle = base - self.base_angle_offset

        self.shoulder_pub.publish(Float64(data=_shoulder_angle))
        self.elbow_pub.publish(Float64(data=_elbow_angle))
        self.base_pub.publish(Float64(data=_base_angle))

    def solve_arm_angles(self, pt):
        '''solve_arm_angles((x, y)) -> Elbow and shoulder angles
        This computes the angles given a point in the SCARA plane

        2DOF has a closed form solution, this method only slightly extends to higher DOF, where there is not a closed form

        Returns None if no solution exists.
        You should *NEVER* get the numpy "Invalid input in arccos" or a NaN output - if you do, something went wrong.'''
        x, y = pt

        distance = np.sqrt((x**2) + (y**2))
        if distance > (self.shoulder_length + self.elbow_length):
            rospy.logwarn("Target arm position out of bounds")
            return None
        elif(y > 0.160):
            rospy.logwarn('Arm reaching too high - some links will collide near the base');
            return None
        elif(y < -0.105):
            rospy.logwarn('Arm reaching too low - possible collision with the ground');
            return None
        elif(x < 0.025):
            rospy.logwarn('Arm is going to collide with itself');
            return None
        elif(y < 0.00 and x < 0.185):
            rospy.logwarn('Arm is going to collide with the chassis');
            return None
        elif(y > 0.080 and x < 0.050):
            rospy.logwarn('Arm is going to collide with its back-carriage');
            return None

        # u-arm (Elbow-Angle wrt ground)
        c2 = ((self.elbow_length**2 + self.shoulder_length**2) - (x**2 + y**2))/(2 * self.elbow_length * self.shoulder_length)
        s2 = np.sqrt(1 - c2**2)

        c1 = (self.shoulder_length * x - self.elbow_length * (x * c2 + y * s2))/(self.shoulder_length**2 + ((self.elbow_length * s2)**2 + (self.elbow_length * c2)**2) - 2 * self.shoulder_length * self.elbow_length * c2)
        shoulder_angle = np.arccos(c1)
        elbow_angle = shoulder_angle + np.arccos(c2)

        if (abs(x - (self.shoulder_length * np.arccos(shoulder_angle) - self.elbow_length * np.arccos(elbow_angle))) > 10**-6 or abs(y - (self.shoulder_length * np.sin(shoulder_angle) - self.elbow_length * np.sin(elbow_angle)) > 10**-6)):
            shoulder_angle =  -shoulder_angle
            elbow_angle = shoulder_angle + np.arccos(c2)

        return (shoulder_angle, elbow_angle)

    def solve_base_angle(self, pt):
        '''Solve base angle given a pt'''
        x, y = pt
        angle = np.arctan2(y, x)
        if (np.fabs(angle) > np.pi / 3):
            rospy.logwarn('Base will rotate out of bounds and collide with rest of robot')
            return None

        return angle


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