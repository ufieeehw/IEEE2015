#!/usr/bin/env python

##All translations are in meters all rotations are in radians##
import roslib
roslib.load_manifest('ieee2015_tf_broadcaster')

import rospy
import numpy as np
import tf
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, Transform
from dynamixel_msgs.msg import JointState

shoulder_angle_offset = 1.06
elbow_angle_offset = 0.65
base_angle_offset = 0.0

tf_broad = tf.TransformBroadcaster()

global last_shoulder_servo_position
last_shoulder_servo_position = 0.0

def handle_camera_pose(msg):
    translation = (msg.pose.position.x, msg.pose.position.y, 0)#assuming forward_camera is in the slot closest to edge
    rotation = tf.transformations.quaternion_from_euler(0, 0, msg.pose.orientation.z)
    time = rospy.Time.now()

    tf_broad.sendTransform(translation, rotation, time, "/forward_camera", "/course")


def robot_center(msg):
    translation = (0.0, 0.0, -0.143) # assuming forward_camera is in the slot closest to edge
    rotation = tf.transformations.quaternion_from_euler(0, 0, 0)
    time = rospy.Time.now()

    tf_broad.sendTransform(translation, rotation, time, "/robot", "/forward_camera")

def read_base_servo(msg):
    global last_base_servo_position
    last_base_servo_position = msg.current_pos + base_angle_offset
    translation = (0.0, 0.0, 0.0) #assuming forward_camera is in the slot closest to edge
    rotation = tf.transformations.quaternion_from_euler(0, 0, last_base_servo_position) #rotates about the y axis to make the z axis along the motor, servo rotation value is sent to z rotation
    time = rospy.Time.now()
    
    tf_broad.sendTransform(translation, rotation, time, "/base_servo", "/robot")
			  

def read_shoulder_servo(msg):
    global last_shoulder_servo_position
    last_shoulder_servo_position = (msg.current_pos + shoulder_angle_offset)
    translation = (0.02073, 0, 0.07358) 
    rotation = tf.transformations.quaternion_from_euler(0.0, np.pi/2-last_shoulder_servo_position, 0.0) #may include a constant shift to follow axis reference conventions, will have a published angle from shoulder servo
    time = rospy.Time.now() 

    tf_broad.sendTransform(translation, rotation, time, "/shoulder_servo", "/base_servo")


def read_elbow_servo(msg):
    global last_elbow_servo_position
    global last_base_servo_position
    last_elbow_servo_position = -(msg.current_pos + elbow_angle_offset)
    translation = (0, 0, 0.148) #assume the servo is located at the elbow for simplicity  
    rotation = tf.transformations.quaternion_from_euler(0, np.pi/2-last_elbow_servo_position + last_shoulder_servo_position, 0) #may include a constant shift to follow axis reference conventions, will have a published angle from elbow servo
    time = rospy.Time.now() 

    tf_broad.sendTransform(translation, rotation, time, "/elbow_servo", "/shoulder_servo")

    send_wrist_joint(msg)


def send_wrist_joint(msg): #end-effector always parallel to the floor, angle reationship between the wrist joint and elbow reference fram  
    '''Called in read_elbow_servo because it only depends on elbow'''
    global last_elbow_servo_position
    global last_base_servo_position
    translation = (-0.160, 0, 0) 
    rotation = tf.transformations.quaternion_from_euler(0, -(-last_elbow_servo_position+last_shoulder_servo_position)+last_shoulder_servo_position-np.pi/2, 0) #angle offset applied to published servo value
    time = rospy.Time.now()
   
    tf_broad.sendTransform(translation, rotation, time, "/wrist_joint", "/elbow_servo")


def read_wrist_servo(msg):
    global last_wrist_servo_position
    last_wrist_servo_position = msg.current_pos
    translation = (0, 0, 0.03442) #constant offset 
    rotation = tf.transformations.quaternion_from_euler(last_wrist_servo_position,0 ,0) #published by end servo
    time = rospy.Time.now() 

    tf_broad.sendTransform(translation, rotation, time, "/wrist_servo", "/wrist_joint")

def read_end_effector_center(msg): 
    translation = (-0.034,0,0) #it will be a slight vertical(z) shift down
    rotation = (0,0,0) 
    time = rospy.Time.now()
   
    tf_broad.sendTransform(translation, rotation, time, "/end_effector_center", "/wrist_servo")

def read_end_effector_card_picker(msg): 
    translation = (-0.09, 0, 0.05) 
    rotation = (0,0,0) 
    time = rospy.Time.now()
   
    tf_broad.sendTransform(translation, rotation, time, "/card_picker", "/end_effector_center")


def read_end_effector_simon_poker(msg): 
    translation = ( -0.092, 0.03,0.052 ) 
    rotation = (0,0,0) 
    time = rospy.Time.now()
   
    tf_broad.sendTransform(translation, rotation, time, "/simon_poker", "/end_effector_center")


def read_end_effector_large_pincher(msg): 
    translation = ( -0.07, -0.035, 0.01) 
    rotation = ()
    time = rospy.Time.now()
   
    tf_broad.sendTransform(translation, rotation, time, "/large_pincher", "/end_effector_center")

def read_end_effector_small_pincher(msg): 
    translation = ( -0.07, 0.035, 0.01) 
    rotation = () 
    time = rospy.Time.now()
   
    tf_broad.sendTransform(translation, rotation, time, "/small_pincher", "/end_effector_center")

def read_end_camera(msg):
    translation = ( 0, -0.234, -0.065) #constant offset 
    rotation = tf.transformations.quaternion_from_euler(0, 0, 0) #follows joint conventions constant rotation of wrist joint always points straight down
    time = rospy.Time.now() 

    tf_broad.sendTransform(translation, rotation, time, "/arm_camera", "/end_effector_center")

''' Toys are pubished from computer vision '''
'''
def read_toy_etch(msg): 
    translation = () 
    rotation = () 
    time = rospy.Time.now()
   
    tf_broad.sendTransform(translation, rotation, time, "/toy_etch", "/arm_camera")
'''

'''
def read_toy_cards(msg): 
    translation = () 
    rotation = () 
    time = rospy.Time.now()
   
    tf_broad.sendTransform(translation, rotation, time, "/toy_cards", "/arm_camera")
'''

'''
def read_toy_rubix(msg): 
    translation = () 
    rotation = () 
    time = rospy.Time.now()
   
    tf_broad.sendTransform(translation, rotation, time, "/toy_rubix", "/arm_camera")
'''

'''
def read_toy_simon(msg): 
    translation = () 
    rotation = () 
    time = rospy.Time.now()
   
    tf_broad.sendTransform(translation, rotation, time, "/toy_simon", "/arm_camera")
'''

if __name__ == '__main__':
    rospy.init_node('ieee2015_tf_broadcaster')

    rospy.Subscriber('pose', PoseStamped, handle_camera_pose)
    rospy.Subscriber('base_controller/state', JointState, read_base_servo)
    rospy.Subscriber('shoulder_controller/state', JointState, read_shoulder_servo)
    rospy.Subscriber('elbow_controller/state', JointState, read_elbow_servo)
    rospy.Subscriber('wrist_controller', Float64, read_wrist_servo)

rospy.spin()
