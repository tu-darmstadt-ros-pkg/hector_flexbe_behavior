#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from hector_perception_msgs.msg import DetectObjectAction, DetectObjectGoal
from flexbe_core.proxy import ProxyTransformListener
from geometry_msgs.msg import PoseStamped

'''
Created on 15.06.2015

@author: Philipp Schillinger
'''

class GetPipePose(EventState):
	'''
	># centerpose	PoseStamped()		Pose of centered pipe.
	#> pipepose	PoseStamped()		Pose of desired pipe.
	'''
	TOP_LEFT = 1
	TOP_RIGHT = 2
	CENTER = 3
	DOWN_LEFT = 4
	DOWN_RIGHT = 5

	def __init__(self, choice):
		'''
		Constructor
		'''
		super(GetPipePose, self).__init__(outcomes=['succeeded'], input_keys = ['centerpose'], output_keys = ['pipepose'])
		self._choice = choice
		
		self._tf = ProxyTransformListener().listener()
		self._succeeded = False
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		if self._succeeded:
			return 'succeeded'
		
			
	def on_enter(self, userdata):
		tempPose = PoseStamped()
		tempPose = self._tf.transformPose('base_link', userdata.centerpose)
		
		if self._choice == GetPipePose.TOP_LEFT:
			tempPose.pose.position.y = tempPose.pose.position.y + 0.105
			tempPose.pose.position.z = tempPose.pose.position.z + 0.105
		if self._choice == GetPipePose.TOP_RIGHT:
			tempPose.pose.position.y = tempPose.pose.position.y - 0.105
			tempPose.pose.position.z = tempPose.pose.position.z + 0.105
		if self._choice == GetPipePose.DOWN_RIGHT:
			tempPose.pose.position.y = tempPose.pose.position.y + 0.105
			tempPose.pose.position.z = tempPose.pose.position.z - 0.105
		if self._choice == GetPipePose.DOWN_LEFT:	
			tempPose.pose.position.y = tempPose.pose.position.y - 0.105
			tempPose.pose.position.z = tempPose.pose.position.z - 0.105

		userdata.pipepose = tempPose
		
		self._succeeded = True
			

