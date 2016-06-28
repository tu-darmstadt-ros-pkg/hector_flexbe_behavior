#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient, ProxyPublisher

from hector_move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionPath
from rospy import Time


'''
Created on 15.06.2015

@author: Philipp Schillinger
'''

class MoveAlongPath(EventState):
	'''
	Lets the robot move along a given path.

	># path		PoseStamped[]			Array of Positions of the robot.
	># speed	float				Speed of the robot

	<= reached 					Robot is now located at the specified waypoint.
	<= failed 					Failed to send a motion request to the action server.
	'''

	def __init__(self):
		'''
		Constructor
		'''
		super(MoveAlongPath, self).__init__(outcomes=['reached', 'failed'], input_keys=['path','speed'])
		
		self._failed = False
		self._reached = False

		self._pathTopic = '/controller/path'
		self._pub = ProxyPublisher({self._pathTopic: MoveBaseActionPath})
		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		
		if self._failed:
			return 'failed'
		if self._reached:
			return 'reached'

		

			
	def on_enter(self, userdata):
		
		self._path = MoveBaseActionPath()
		self._path.goal.target_path.poses = userdata.path.poses

		self._failed = False

		
		self._pub.publish(self._pathTopic, self._path)
		self._reached = True
		
			

	def on_stop(self):
			pass

	def on_exit(self, userdata):
		
			pass

	def on_pause(self):
		self._move_client.cancel(self._action_topic)

	def on_resume(self, userdata):
		self.on_enter(userdata)
