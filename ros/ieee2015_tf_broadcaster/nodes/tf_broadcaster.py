#!/usr/bin/env python

##All translations are in meters all rotations are in radians##
import roslib
roslib.load_manifest('ieee2015_tf_broadcaster')

import rospy
import numpy as np
import tf

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
    rotation = tf.transformations.quaternion_from_euler(0, np.pi / 2, last_base_servo_position) #rotates about the y axis to make the z axis along the motor, servo rotation value is sent to z rotation
    time = rospy.Time.now()
    
    tf_broad.sendTransform(translation, rotation, time, "/base_servo", "/robot")
			  

def read_shoulder_servo(msg):
    global last_shoulder_servo_position
    last_shoulder_servo_position = (msg.current_pos + shoulder_angle_offset)
    translation = (-0.07358, 0, 0.02073) 
    rotation = tf.transformations.quaternion_from_euler(0.0, np.pi/2-last_shoulder_servo_position, 0.0) #may include a constant shift to follow axis reference conventions, will have a published angle from shoulder servo
    time = rospy.Time.now() 

    tf_broad.sendTransform(translation, rotation, time, "/shoulder_servo", "/base_servo")


def read_elbow_servo(msg):
    global last_elbow_servo_position
    global last_base_servo_position
    last_elbow_servo_position = -(msg.current_pos + elbow_angle_offset)
    translation = (-0.148, 0, 0) #assume the servo is located at the elbow for simplicity  
    rotation = tf.transformations.quaternion_from_euler(0, -last_elbow_servo_position + last_shoulder_servo_position, 0) #may include a constant shift to follow axis reference conventions, will have a published angle from elbow servo
    time = rospy.Time.now() 

    tf_broad.sendTransform(translation, rotation, time, "/elbow_servo", "/shoulder_servo")

    send_wrist_joint()


def send_wrist_joint(): #end-effector always parallel to the floor, angle reationship between the wrist joint and elbow reference fram  
    '''Called in read_elbow_servo because it only depends on elbow'''
    translation = (-0.160, 0, 0) 
    rotation = tf.transformations.quaternion_from_euler(0, 0, last_elbow_servo_position) #angle offset applied to published servo value
    time = rospy.Time.now()
   
    tf_broad.sendTransform(translation, rotation, time, "/wrist_joint", "/elbow_servo")


def read_wrist_servo(msg):
    global last_wrist_servo_position
    last_wrist_servo_position = msg.current_pos
    translation = (-0.03442, 0, 0) #constant offset 
    rotation = tf.transformations.quaternion_from_euler(np.pi / 2, 0, last_wrist_servo_position) #published by end servo
    time = rospy.Time.now() 

    tf_broad.sendTransform(translation, rotation, time, "/wrist_servo", "/wrist_joint")


def read_end_camera(msg):
    translation = (0.0625, 0, 0.0034) #constant offset 
    rotation = tf.transformations.quaternion_from_euler(0, 0, 0) #follows joint conventions constant rotation of wrist joint always points straight down
    time = rospy.Time.now() 

    tf_broad.sendTransform(translation, rotation, time, "/arm_camera", "/wrist_servo")


'''
def read_end_effector(msg): 
	translation = () #it will be a slight vertical(z) shift down
	rotation = () #joint value(z)
	time = rospy.Time.now()
   
    tf_broad.sendTransform(translation, rotation, time, "/end_effector", "/end_joint")
'''

if __name__ == '__main__':
    rospy.init_node('ieee2015_tf_broadcaster')

    rospy.Subscriber('pose', PoseStamped, handle_camera_pose)
    rospy.Subscriber('base_controller/state', JointState, read_base_servo)
    rospy.Subscriber('shoulder_controller/state', JointState, read_shoulder_servo)
    rospy.Subscriber('elbow_controller/state', JointState, read_elbow_servo)
    rospy.Subscriber('wrist_controller/state', JointState, read_wrist_servo)

rospy.spin()
