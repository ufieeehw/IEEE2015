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

max_linear_vel = 1 # m/s
max_linear_acc = max_linear_vel # m/s^2

max_angular_vel = 2 # rad/s
max_angular_acc = max_angular_vel # rad/s^2 
# (Jason says this is just called angular acceleration, # I call it angcelleration)

def print_in(f):
    '''Shitty decorator for printing function business'''
    print("Defining " + f.func_name)
    def print_on_entry(*args, **kwargs):
        print("Executing " + f.func_name)
        result = f(*args, **kwargs)
        print("Returning " + str(result))
        return(result)
    return(print_on_entry)


def xyzw_array(quaternion):
    '''Convert quaternion to non-shit array'''
    xyzw_array = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    return(xyzw_array)


class Controller(object):
    '''Controller object 
    See Jacob Panikulam or Aaron Marquez for questions

    Function:
        - Determine the ideal velocity for the current position, solve for wheel rotation speeds
            via linear least squares, using the linear relations presented in [1]
        V_y = (V_0 + V_1 + V_2 + V_3) / 4
        V_x = (V_0 - V_1 + V_2 - V_3) / 4
        V_theta  = (V_0 + V_1 - V_2 - V_3)/4
        V_error = (V_0 - V_1 - V_2 + V_3)/4

    Notes:
        Special glory to Lord Forrest Voight, creator of the universe
        Greater thanks to the incredible, brilliant and good-looking Khaled Hassan, for his unyielding mentorship
        and love for Burrito Bros


    Bibliography:
        [1] http://www2.informatik.uni-freiburg.de/~grisetti/teaching/ls-slam/lectures/pdf/ls-slam-03-hardware.pdf
    '''
    def __init__(self):
        '''Initialize Controller Object'''
        rospy.init_node('vehicle_controller')

        # Twist pub
        twist_topic = 'desired_velocity'
        self.twist_pub = rospy.Publisher(twist_topic, Twist, queue_size=1) 
        self.mecanum_pub = rospy.Publisher('mecanum_speeds', Mecanum, queue_size=1)
        
        # Initializations to avoid weird desynchronizations
        self.des_position = None
        self.des_yaw = None
        # Don't want to have an a-priori position
        self.position = None
        self.yaw = None

        # Create the 4x4 mecanum transformation matrix
        mecanum_matrix = np.matrix([
            [+1, +1, +1, +1],
            [+1, -1, +1, -1],
            [+1, +1, -1, -1],
            [+1, -1, -1, +1],  # This is the error row (May not be necessary)
        ], dtype=np.float32) / 4.0  # All of the rows are divided by 4

        # Compute the left-inverse once
        self.left_inverse = (mecanum_matrix.T * mecanum_matrix).I * mecanum_matrix.T

        # Current pose sub
        self.pose_sub = rospy.Subscriber('pose', PoseStamped, self.got_pose)
        self.desired_pose_sub = rospy.Subscriber('desired_pose', PoseStamped, self.got_desired_pose)

    def send_twist(self, (xvel,yvel), angvel):
        '''Generate twist message'''
        self.twist_pub.publish(
            Twist(
                linear=Vector3(xvel, yvel, 0),
                angular=Vector3(0, 0, angvel),  # Radians
            )
        )

    def send_mecanum(self, (xvel, yvel), angvel):
        '''Convert a desired linear and angular velocity vector into a wheel speed solution
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
            Q.E.D.

        Notes:
            This approach is a slight departure from Forrest and Khaled's method from last year

            **** WHY DID WE NOT JUST USE LEAST SQUARES? **** 
              We precomputed the left-inverse instead of using the numpy least-squares function because
               numpy's least squares must compute the cholesky decomposition each time it is called, and 
               then solve. This way, we still have to do an expensive inversion, but only once.
               In my opinion (Jacob) this is better suited to the real-time application at hand.
               And moreover, one that we understand
        '''
        v_target = np.matrix([xvel, yvel, angvel, 0.0], np.float32).T
        mecanum_speeds = self.left_inverse * v_target 
        wheel_speeds = Mecanum(
            vel_0=mecanum_speeds[0],
            vel_1=mecanum_speeds[1],
            vel_2=mecanum_speeds[2],
            vel_3=mecanum_speeds[3],
        )
        self.mecanum_pub.publish(wheel_speeds)

    def norm_angle_diff(self, ang_1, ang_2):
        '''norm_angle_diff(ang_1, ang_2)
        -> Normalized angle difference, constrained to range [-pi, pi]
        '''
        return (ang_1 - ang_2 + np.pi) % (2 * np.pi) - np.pi

    def unit_vec(self, v):
        '''unit_vec(v)'''
        norm = np.linalg.norm(v)
        if norm == 0:
            return(v)
        return np.array(v)/np.linalg.norm(v)

    def vec_diff(self, v1, v2):
        '''norm(v1 - v2)'''
        assert isinstance(v1, np.array)
        assert isinstance(v2, np.array)
        diff = np.linalg.norm(v1 - v2)
        return(diff)

    def sign(self, x):
        '''return sign of x -> -1, 1 or 0'''
        if x > 0: return 1
        elif x < 0: return -1
        else: return 0

    def got_pose(self, msg):
        '''recieve current pose of robot

        Function:
            Attempts to construct a velocity solution using a k*sqrt(error) controller
            This tries to guarantee a constant acceleration

        Note:
            Right now, this does not require velocity feedback, only pose
            This SHOULD include velocity feedback, once we have it

        Todo:
            Add speed-drop for the case where position feedback is lost
            This will probably have to be done in a separate thread

        Velocity calcluation should be done in a separate thread
         this thread should have an independent information "watchdog" timing method
        '''
        if (self.des_pose is None) or (self.des_yaw is None):
            return

        # World frame position
        self.position = np.array([msg.pose.position.x, msg.pose.position.y])
        self.yaw = tf_trans.euler_from_quaternion(xyzw_array(msg.pose.orientation))[2]

        position_error = self.des_position - self.position
        yaw_error = self.norm_angle_diff(self.des_yaw, self.yaw)
        # Determines the linear speed necessary to maintain a consant backward acceleration
        linear_speed = min(
                            math.sqrt(2 * np.linalg.norm(position_error) * max_linear_acc),
                            max_linear_vel
                        )
        # Determines the angular speed necessary to maintain a constant angular acceleration 
        #  opposite the direction of motion
        angular_speed = min(
                            math.sqrt(2 * abs(yaw_error) * max_angular_acc), 
                            max_angular_vel
                        )

        # Provide direction for both linear and angular velocity
        desired_vel = linear_speed * self.unit_vec(position_error)
        desired_angvel = angular_speed * self.sign(yaw_error)

        # Unit vectors that determine the right-handed coordinate system of the roobt in the world frame
        forward = np.array([math.cos(self.yaw), math.sin(self.yaw)])
        left = np.array([math.cos(self.yaw + np.pi/2), math.sin(self.yaw + np.pi/2)])

        # Send twist if above error threshold
        if (np.fabs(yaw_error) > 0.01) or (np.linalg.norm(position_error) > 0.01):
            x_vel = forward.dot(desired_vel)
            y_vel = left.dot(desired_vel)

            # Send the raw x, y, w desired velocity vector
            # Alone, this line does nothing
            self.send_twist((x_vel, y_vel), desired_angvel) 

            # Compute the actual wheel speeds (This is what makes the robot physically move)
            self.send_mecanum((x_vel, y_vel), desired_angvel)  
    
    def got_desired_pose(self, msg):
        '''Recieved desired pose message
        Figure out how to do this in a separate thread
        So we're not depending on a message to act
        #LearnToThreading
        (That's a hashtag)
        '''
        self.des_position = np.array([msg.pose.position.x, msg.pose.position.y])
        self.des_yaw = tf_trans.euler_from_quaternion(xyzw_array(msg.pose.orientation))[2]

if __name__ == '__main__':
    controller = Controller()
    rospy.spin()