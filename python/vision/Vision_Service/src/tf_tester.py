#!/usr/bin/env python

##All translations are in meters all rotations are in radians##
import roslib
roslib.load_manifest('ieee2015_tf_broadcaster')

import rospy
import numpy as np
import tf

from geometry_msgs.msg import PoseStamped, Transform
from dynamixel_msgs.msg import JointState


tf_broad = tf.TransformBroadcaster()

def handle_camera_pose():
    translation = (0, 0, 0)#assuming camera is in the slot closest to edge
    print translation
    rotation = tf.transformations.quaternion_from_euler(0,0,0)
    print rotation
    time = rospy.Time.now()

    tf_broad.sendTransform(translation, rotation, time, "/camera", "/course")

def robot_center():
    translation = (0, 0, -0.143) # assuming forward_camera is in the slot closest to edge
    rotation = tf.transformations.quaternion_from_euler(0, np.pi/2, 0)
    time = rospy.Time.now()

    tf_broad.sendTransform(translation, rotation, time, "/robot", "/camera")

def read_spinner_servo():
    global last_spinner_servo_position
    #last_spinner_servo_position = msg.current_pos
    translation = (0, 0, 0.0185) #assuming camera is in the slot closest to edge
    rotation = tf.transformations.quaternion_from_euler(0, np.pi/2, 0) #rotates about the y axis to make the z axis along the motor, servo rotation value is sent to z rotation
    time = rospy.Time.now()
    
    tf_broad.sendTransform(translation, rotation, time, "/spinner_servo", "/robot")
			  

def read_shoulder_servo():
    global last_shoulder_servo_position
    #last_shoulder_servo_position = msg.current_pos
    translation = (-0.07358, 0, -0.02073) 
    rotation = tf.transformations.quaternion_from_euler(0, 2*np.pi, 0) #may include a constant shift to follow axis reference conventions, will have a published angle from shoulder servo
    time = rospy.Time.now() 

    tf_broad.sendTransform(translation, rotation, time, "/shoulder_servo", "/spinner_servo")

def read_elbow_servo():
    global last_elbow_servo_position
    #last_elbow_servo_position = msg.current_pos
    translation = (-0.148, 0, 0) #assume the servo is located at the elbow for simplicity  
    rotation = tf.transformations.quaternion_from_euler(0, -np.pi/2, 0) #may include a constant shift to follow axis reference conventions, will have a published angle from elbow servo
    time = rospy.Time.now() 

    tf_broad.sendTransform(translation, rotation, time, "/elbow_servo", "/shoulder_servo")


def read_wrist_joint(): #end-effector always parallel to the floor, angle reationship between the wrist joint and elbow reference fram  
    translation = (-0.160, 0, 0) 
    rotation = tf.transformations.quaternion_from_euler(0, 0, 0) #angle offset applied to published servo value
    time = rospy.Time.now()
   
    tf_broad.sendTransform(translation, rotation, time, "/wrist_joint", "/elbow_servo")

def read_end_servo():
    global last_end_servo_position
    #last_end_servo_position = msg.current_pos
    translation = (-0.03442, 0, 0) #constant offset 
    rotation = tf.transformations.quaternion_from_euler(np.pi/2, 0, 0) #published by end servo
    time = rospy.Time.now() 

    tf_broad.sendTransform(translation, rotation, time, "/end_servo", "/wrist_joint")

def read_end_camera():
    translation = (0.0625, 0, 0.0034) #constant offset 
    rotation = tf.transformations.quaternion_from_euler(0, 0, 0) #follows joint conventions constant rotation of wrist joint always points straight down
    time = rospy.Time.now() 
    print translation
    tf_broad.sendTransform(translation, rotation, time, "/end_camera", "/end_servo")


'''
def read_end_effector(msg): 
	translation = () #it will be a slight vertical(z) shift down
	rotation = () #joint value(z)
	time = rospy.Time.now()
   
    tf_broad.sendTransform(translation, rotation, time, "/end_effector", "/end_joint")
'''

if __name__ == '__main__':
    rospy.init_node('ieee2015_tf_broadcaster')
   
    while not rospy.is_shutdown():

        handle_camera_pose()
        robot_center()
        read_spinner_servo()
        read_shoulder_servo()
        read_elbow_servo()
        read_wrist_joint()
        read_end_servo()
        read_end_camera()
'''
    #sim = rospy.get_param('~sim', "N")
    if(sim == 'Y'):
        prefix = 'sim_'
    else:
        prefix = ''

    rospy.Subscriber('/{0}pose'.format(prefix), PoseStamped, handle_camera_pose)
    rospy.Subscriber('/{0}spinner_controller/state'.format(prefix), JointState, read_spinner_servo)
    rospy.Subscriber('/{0}shoulder_controller/state'.format(prefix), JointState, read_shoulder_servo)
    rospy.Subscriber('/{0}elbow_controller/state'.format(prefix), JointState, read_elbow_servo)
    rospy.Subscriber('/{0}end_controller/state'.format(prefix), JointState, read_end_servo)
'''

