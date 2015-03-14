#!/usr/bin/env python
from __future__ import division
## Math
import numpy as np
import math
## Ros/tools
import tf
from tf import transformations as tf_trans
import rospy
## Ros msgs
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, Twist, TwistStamped, Vector3
from ieee2015_msgs.msg import Mecanum
from xmega_connector.srv import * #Echo, EchoRequest, EchoResponse, SetWheelSpeeds

class Controller(object):
    def __init__(self):
        '''Initialize Controller Object'''
        rospy.init_node('mecanum_controller')
    
        # Create the 4x4 mecanum transformation matrix
        mecanum_matrix = np.matrix([
            [+1, +1, +1, +1],  # Unitless! Shooting for rad/s
            [+1, -1, +1, -1],  # Unitless! Shooting for rad/s
            [+1, +1, -1, -1],  # Unitless! Shooting for rad/s
            [+1, -1, -1, +1],  # This is the error row (May not be necessary)
        ], dtype=np.float32) / 4.0  # All of the rows are divided by 4

        # Compute the left-inverse once
        self.left_inverse = (mecanum_matrix.T * mecanum_matrix).I * mecanum_matrix.T

        # Twist subscriber
        self.twist_sub = rospy.Subscriber('/twist', Twist, self.got_twist)
        # We do not currently use the Mecanum publisher, instead only the service is called
        self.mecanum_pub = rospy.Publisher('/mecanum_speeds', Mecanum, queue_size=1)
        rospy.loginfo("Attempting to find set_wheel_speeds service")
        self.wheel_speed_proxy = rospy.ServiceProxy('/xmega_connector/set_wheel_speeds', SetWheelSpeeds)

    def got_twist(self, twist_msg):
        desired_action = np.array([twist_msg.linear.x, twist_msg.linear.y, twist_msg.angular.z, 0.0], dtype=np.float32)
        self.send_mecanum(desired_action)

    def send_mecanum(self, desired_action):
        '''Convert a desired linear and angular velocity vector into a wheel speed solution

        Equations from [1]
        V_y = (V_0 + V_1 + V_2 + V_3) / 4
        V_x = (V_0 - V_1 + V_2 - V_3) / 4
        V_theta  = (V_0 + V_1 - V_2 - V_3) / 4
        V_error = (V_0 - V_1 - V_2 + V_3) / 4

        Function:
            (If you don't understand this, I suggest the Khan Academy Linear Algebra series
                very short and sweet)
            Precompute the left-inverse of the mecanum characteristic matrix
            Multiply that by the desired velocity
        ****This is equivalent to the least-squares solution****
        Proof:  
            Motivation:
                We would like to find a vector x* such that |b - x*| is minimized
                 (In this case, b is the 'vector' of desired [xvel, yvel, angvel])
                The vector that minimizes |b - x*| is the projection of b onto the column
                space (Call this subspace V) of A.

            So...
                Ax* = b_proj
                (Ax* - b) is orthogonal to V therefore,
                (Ax* - b) is a member of the null-space of A.T (Also called the left null-space)

            Then by definition of null-space,
                A.T(x* - b) = <0>
                Where <0> is the zero vector

            Distribute:
                (A.T * Ax*) - (A.T * <b>) = <0>
                A.T * Ax* = A.T * <b>
                x* = (A.T * A).I ( A.T * <b>)  
                For those keeping track at home, (A.T * A).I * A.T is the Moore-Penrose Left Psuedo-inverse of A
            Q.E.D.

        Notes:
            This approach is a slight departure from Forrest and Khaled's method from last year

        FAQ:
            Q: Why did we not just use Numpy's least squares implementation? (numpy.linalg.lstsq)?
            A:  We precomputed the left-inverse instead of using the numpy least-squares function because
                numpy's least squares must compute the cholesky decomposition each time it is called, and 
                then solve. This way, we still have to do an expensive inversion, but only once.
                In my opinion (Jacob) this is better suited to the real-time application at hand.
                And moreover, one that we understand

        Bibliography:
            [1] http://www2.informatik.uni-freiburg.de/~grisetti/teaching/ls-slam/lectures/pdf/ls-slam-03-hardware.pdf

        '''
        v_target = np.matrix(desired_action).T
        mecanum_speeds = self.left_inverse * v_target 
        wheel_speeds = [mecanum_speeds[0], mecanum_speeds[1], mecanum_speeds[2], mecanum_speeds[3]]
        self.wheel_speed_proxy(*wheel_speeds)


if __name__=='__main__':
    controller = Controller()
    rospy.spin()