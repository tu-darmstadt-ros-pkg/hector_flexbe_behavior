#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

'''
Created on 15.06.2015

@author: Philipp Schillinger
'''

class SelectPipePose(EventState):
	'''
	Selects one of the detected pipes.

	># object_pose		PoseStamped()		Pose of the percepted pipe object.
	># object_data		float[] 			Data associated with the percept.
	># pose_choice 		int 				Index of the desired pose.

	#> pipe_pose		PoseStamped()		Pose of desired pipe.

	<= selected 							Calculated the desired pipe pose.
	<= out_of_range 						No more pipes available.
	'''

	def __init__(self):
		'''
		Constructor
		'''
		super(SelectPipePose, self).__init__(outcomes=['selected', 'out_of_range'],
									input_keys = ['object_pose', 'object_data', 'pose_choice'],
									output_keys = ['pipe_pose'])
		self._out_of_range = False
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		if self._out_of_range:
			return 'out_of_range'

		return 'selected'
		
			
	def on_enter(self, userdata):
		pipe_pose = userdata.object_pose

		i = userdata.pose_choice * 3
		if i < len(userdata.object_data):
			self._out_of_range = False
			pipe_pose.pose.position.x = userdata.object_data[i]
			pipe_pose.pose.position.y = userdata.object_data[i+1]
			pipe_pose.pose.position.z = userdata.object_data[i+2]
		else:
			self._out_of_range = True

		userdata.pipe_pose = pipe_pose
			

