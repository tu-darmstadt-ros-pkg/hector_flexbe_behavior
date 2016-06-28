#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from hector_perception_msgs.msg import DetectObjectAction, DetectObjectGoal

'''
Created on 15.06.2015

@author: Philipp Schillinger
'''

class PipeDetectionState(EventState):
	'''
	Triggers pipe detection to find the location of the pipes

	-- max_attempts 	int 			Maximum number of re-running attempts.
										0 means unlimitied.

	<= found 							Found the pipe location.
	<= unknown 							Did not detect any pipes.

	'''


	def __init__(self, max_attempts = 1):
		'''
		Constructor
		'''
		super(PipeDetectionState, self).__init__(outcomes=['found', 'unknown'])
		
		self._action_topic = '/hector_five_pipes_detection_node/detect'
		self._client = ProxyActionClient({self._action_topic: DetectObjectAction})

		self._max_attempts = max_attempts
		self._success = False
		self._failed = False
		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		if self._failed:
			return 'unknown'

		if self._client.has_result(self._action_topic):
			result = self._client.get_result(self._action_topic)
			
			if result.detection_success:
				return 'found'
			elif self._max_attempts == 0:
				Logger.loginfo('Did not detect pipes, trying again (unlimited)...')
				self.on_enter(userdata)
			elif self._max_attempts > 1:
				self._max_attempts -= 1
				Logger.loginfo('Did not detect pipes, trying again (%d more times)...' % self._max_attempts)
				self.on_enter(userdata)
			else:
				return 'unknown'


			
	def on_enter(self, userdata):
		self._failed = False

		action_goal = DetectObjectGoal()

		try:
			self._client.send_goal(self._action_topic, action_goal)
		except Exception as e:
			Logger.logwarn('Failed to send pipe detection request:\n%s' % str(e))
			self._failed = True
			

	def on_stop(self):
		try:
			if self._client.is_available(self._action_topic) \
			and not self._client.has_result(self._action_topic):
				self._client.cancel(self._action_topic)
		except:
			# client already closed
			pass

	def on_pause(self):
		self._client.cancel(self._action_topic)

	def on_resume(self, userdata):
		self.on_enter(userdata)
