#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

#from hector_perception_msgs.msg import LookAtTarget
#from hector_perception_msgs.action import LookAt

import hector_perception_msgs.msg

'''
Created on 15.06.2015

@author: Philipp Schillinger
'''

class LookAtPattern(EventState):
	'''
	Specify a pattern for the robots camera to follow

	># pattern		string				The pattern to follow

	<= succeeded 						Camera follows the pattern
	<= failed 						Failed to set pattern

	'''

	def __init__(self):
		'''
		Constructor
		'''
		super(LookAtPattern, self).__init__(outcomes=['succeeded', 'failed'],
											input_keys=['pattern'])
		
		self._action_topic = '/pan_tilt_sensor_head_joint_control/look_at'
		self._client = ProxyActionClient({self._action_topic: hector_perception_msgs.msg.LookAtAction})

		self._failed = False
		self._succeeded = False
		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		if self._failed:
			return 'failed'
		if self._succeeded:
			return 'succeeded'


			
	def on_enter(self, userdata):
		self._failed = False
		self._succeeded = False

		action_goal = hector_perception_msgs.msg.LookAtGoal()
		action_goal.look_at_target.pattern = userdata.pattern

		try:
			self._client.send_goal(self._action_topic, action_goal)
		except Exception as e:
			Logger.logwarn('Failed to send LookAt request with pattern %(pat)s\n%(err)s' % {
				'pat': userdata.pattern,
				'err': str(e)
				
			})
			self._failed = True
		
		Logger.loginfo('Pattern %(pat)s loaded' % { 'pat': userdata.pattern })
		self._succeeded = True
			

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
