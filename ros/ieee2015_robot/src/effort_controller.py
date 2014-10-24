#!/usr/bin/env python

# Ros Imports
import rospy
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates

# Threading and math
# import threading  # Not currently used
import numpy as np

class topic_PID(object):
	'''TODO: Implement PID controller object for responding to an arbitrary state topic with a corresponding effort
	Include:
		Settable gains
		Settable history length (Controls the number of past samples the integral and derivative terms will account for)
	'''
	def __init__(self, subscriber, publisher, history_length=20):
		self.sub_topic = None

	def got_sub(self, data):
		pass


class Effort_Control(object):
	'''Effort Control object
	Use link_state data from Gazebo to generate PID effort controls to give the arm a reasonable positioning

	[1] http://gazebosim.org/tutorials?tut=ros_control&cat=connect_ros
	'''

	def __init__(self):
		rospy.init_node('joint_states_listener')

		# This is the list of joint names that we are interested in
		self.joint_names = ['ieee2015_robot::ArmRot1']
		self.link_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.got_link_states)

	def handle_link_state(self, joint_number, states):
		'''Given a states list and a joint number, operate on the state data'''
		joint_twist = states.twist[joint_number]
		joint_pose = states.pose[joint_number]

	def got_link_states(self, states):
		'''Received link_states messages from gazebo_sim
		The "states" input comes as a LinkStates object, with name, pose and twist lists as attributes.
		The "LinkStates" message looks like:
			states.name = ["ieee2015_robot::ArmRot1", "ieee2015_robot::ArmRot2"] (and so on)
			states.twist = [~twist for ArmRot1~, ~twist for ArmRot2~]
			states.pose =[~pose for ArmRot1~, ~pose for ArmRot2~]
		'''

		# Loop through the target joint names
		for joint_name in self.joint_names:
			joint_number = states.name.index(joint_name)
			self.handle_link_state(joint_number)


if __name__ == '__main__':
	effort_control = Effort_Control()
	rospy.spin()