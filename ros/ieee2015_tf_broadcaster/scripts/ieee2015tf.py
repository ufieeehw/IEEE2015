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
    translation = (msg.pose.position.x, msg.pose.position.y, 0)
    rotation = (msg.pose.orientation.x,msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
    time = rospy.Time.now()

    tf_broad.sendTransform(translation, rotation, time, "/camera", "/course")

def read_chassis_center_position(msg):
	translation = () #enter distance offset from camera lense to center of the base of the robot
	rotation = (0, 0, 0) #there is no rotation because sliding the reference frame to the center of the robot complets the Transform
	time = rospy.Time.now()
	
	tf_broad.sendTransform(translation, rotation, time, "/chassis_center", "/camera")

def read_spinner_servo(msg):
	translation = () #it will be a slight vertical shift up
	rotation = () #may include a constant shift to follow axis reference conventions, will have a published angle from spinner servo
	time = rospy.Time.now()
    
    tf_broad.sendTransform(translation, rotation, time, "/spinner_servo", "/chassis_center")			  

def read_spinner_joint(msg): #needed only if there is an offset from the servo and the spiner joint
	translation = () #it will be a slight vertical shift down
	rotation = () #angle offset applied to published servo value
	time = rospy.Time.now()
    
    tf_broad.sendTransform(translation, rotation, time, "/spinner_joint", "/spinner_servo")

def read_shoulder_servo(msg):
	translation = () #it will slide up and to the right or left
	rotation = () #may include a constant shift to follow axis reference conventions, will have a published angle from shoulder servo
	time = rospy.Time.now() 

	tf_broad.sendTransform(translation, rotation, time, "/shoulder_servo", "/spinner_joint")

def read_shoulder_joint(msg): #needed for the offset from the servo and the shoulder joint
	translation = () #it will be a slight vertical shift down
	rotation = () #angle offset applied to published servo value
	time = rospy.Time.now()
    
    tf_broad.sendTransform(translation, rotation, time, "/shoulder_joint", "/shoulder_servo")

def read_elbow_servo(msg):
	translation = () #it will slide to the right or left
	rotation = () #may include a constant shift to follow axis reference conventions, will have a published angle from elbow servo
	time = rospy.Time.now() 

	tf_broad.sendTransform(translation, rotation, time, "/elbow_servo", "/shoulder_joint")

def read_elbow_joint(msg): #needed for the offset from the servo and the elbow joint
	translation = () #vector: constant magnitude with an angle function
	rotation = () #angle offset applied to published servo value
	time = rospy.Time.now()
    
    tf_broad.sendTransform(translation, rotation, time, "/elbow_joint", "/elbow_servo")

def read_wrist_joint(msg): #end-effector always parallel to the floor, angle reationship between the wrist joint and elbow reference fram  
	translation = () #vector: constant magnitude with an angle function
	rotation = () #angle offset applied to published servo value
	time = rospy.Time.now()
    
    tf_broad.sendTransform(translation, rotation, time, "/wrist_joint", "/elbow_joint")

def read_end_camera(msg):
	translation = () #constant offset 
	rotation = () #follows joint conventions constant rotation of wrist joint always points straight down
	time = rospy.Time.now() 

	tf_broad.sendTransform(translation, rotation, time, "/end_camera", "/wrist_joint")

def read_end_servo(msg):
	translation = () #constant offset 
	rotation = () #published by end servo
	time = rospy.Time.now() 

	tf_broad.sendTransform(translation, rotation, time, "/end_servo", "/end_camera")

def read_end_joint(msg): #needed only if there is an offset from the servo and the end-effector joint
	translation = () #it will be a slight vertical shift down
	rotation = () #angle offset applied to published servo value
	time = rospy.Time.now()
    
    tf_broad.sendTransform(translation, rotation, time, "/end_joint", "/end_servo")

def read_end_effector(msg): 
	translation = () #it will be a slight vertical shift down
	rotation = () #joint value
	time = rospy.Time.now()
    
    tf_broad.sendTransform(translation, rotation, time, "/end_effector", "/end_joint")


    if __name__ == '__main__':
    rospy.init_node('ieee2015_tf_broadcaster')

    sim = rospy.get_param('~sim', "N")
    if(sim == 'Y'):
        prefix = 'sim_'
    else:
        prefix = ''

    rospy.Subscriber('/{0}pose'.format(prefix), PoseStamped, handle_lidar_pose)
    rospy.Subscriber('/{0}pan_controller/state'.format(prefix), JointState, read_pan_position)
    rospy.Subscriber('/{0}tilt_controller/state'.format(prefix), JointState, read_tilt_position)

    rospy.spin()