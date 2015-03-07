#!/usr/bin/env python

##All translations are in meters all rotations are in radians##
import roslib
roslib.load_manifest('ieee2015_tf_broadcaster')

import rospy

import tf

from geometry_msgs.msg import PoseStamped, Transform
from dynamixel_msgs.msg import JointState

last_pan_position = 0.0
last_tilt_position = 0.0
tf_broad = tf.TransformBroadcaster()

def handle_camera_pose(msg):
    translation = (msg.pose.position.x, msg.pose.position.y, 0)#assuming camera is in the slot closest to edge
    rotation = tf.transformations.quaternion_from_euler(msg.pose.orientation.x,msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
    time = rospy.Time.now()

    tf_broad.sendTransform(translation, rotation, time, "/camera", "/course")

def read_spinner_servo(msg):
    global last_spinner_servo_position
    last_spinner_servo_position = msg.current_pos
    translation = (0.13948, 0, -0.0185) #assuming camera is in the slot closest to edge
    rotation = tf.transformations.quaternion_from_euler(0, 1.570796, last_spinner_servo_position) #rotates about the y axis to make the z axis along the motor, servo rotation value is sent to z rotation
    time = rospy.Time.now()
    
    tf_broad.sendTransform(translation, rotation, time, "/spinner_servo", "/camera")			  

def read_shoulder_servo(msg):
    global last_shoulder_servo_position
    last_shoulder_servo_position = msg.current_pos
    translation = (0.02073, 0, 0.04128) 
    rotation = tf.transformations.quaternion_from_euler(1.570796, 0, last_shoulder_servo_position) #may include a constant shift to follow axis reference conventions, will have a published angle from shoulder servo
    time = rospy.Time.now() 

    tf_broad.sendTransform(translation, rotation, time, "/shoulder_servo", "/spinner_servo")

def read_elbow_servo(msg):
    global last_elbow_servo_position
    last_elbow_servo_position = msg.current_pos
    translation = ( 0.148, 0, 0) #assume the servo is located at the elbow for simplicity  
    rotation = tf.transformations.quaternion_from_euler(0, 0, last_elbow_servo_position) #may include a constant shift to follow axis reference conventions, will have a published angle from elbow servo
    time = rospy.Time.now() 

    tf_broad.sendTransform(translation, rotation, time, "/elbow_servo", "/shoulder_servo")


def read_wrist_joint(msg): #end-effector always parallel to the floor, angle reationship between the wrist joint and elbow reference fram  
    translation = (0.5954, 0, 0) 
    rotation = tf.transformations.quaternion_from_euler(0, 0, -last_elbow_servo_position) #angle offset applied to published servo value
    time = rospy.Time.now()
   
    tf_broad.sendTransform(translation, rotation, time, "/wrist_joint", "/elbow_servo")

def read_end_servo(msg):
    global last_end_servo_position
    last_end_servo_position = msg.current_pos
    translation = (0.03442, 0, 0) #constant offset 
    rotation = tf.transformations.quaternion_from_euler(1.570796, 0, last_end_servo_position) #published by end servo
    time = rospy.Time.now() 

    tf_broad.sendTransform(translation, rotation, time, "/end_servo", "/wrist_joint")

def read_end_camera(msg):
    translation = (0.0625, 0, 0.0034) #constant offset 
    rotation = tf.transformations.quaternion_from_euler(0, 0, 0) #follows joint conventions constant rotation of wrist joint always points straight down
    time = rospy.Time.now() 

    tf_broad.sendTransform(translation, rotation, time, "/end_camera", "/end_servo")



#def read_end_effector(msg): 
#	translation = () #it will be a slight vertical shift down
#	rotation = () #joint value
#	time = rospy.Time.now()
   
#    tf_broad.sendTransform(translation, rotation, time, "/end_effector", "/end_joint")


    if __name__ == '__main__':
    rospy.init_node('ieee2015_tf_broadcaster')

    sim = rospy.get_param('~sim', "N")
    if(sim == 'Y'):
        prefix = 'sim_'
    else:
        prefix = ''

    rospy.Subscriber('/{0}pose'.format(prefix), PoseStamped, handle_camera_pose)
    rospy.Subscriber('/{0}spinner_controller/state'.format(prefix), JointState, read_spinner_servo)
    rospy.Subscriber('/{0}shoulder_controller/state'.format(prefix), JointState, read_shoulder_servo)
    rospy.Subscriber('/{0}elbow_controller/state'.format(prefix), JointState, read_elbow_servo)
    rospy.Subscriber('/{0}end_controller/state'.format(prefix), JointState, read_end_servo)

    rospy.spin()
