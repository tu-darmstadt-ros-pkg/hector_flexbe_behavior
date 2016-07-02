#!/usr/bin/env python
import rospy

from flexbe_core.proxy import ProxyActionClient

from flexbe_core import EventState, Logger
from rospy import Time
import time
from hector_move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from dynamic_reconfigure.client import Client




class Explore(EventState):
	'''
	Starts the Exploration Task via /move_base

	># speed			Speed of the robot

	<= succeeded			Exploration Task was successful
	<= failed 			Exploration Task failed

	'''

	def __init__(self):
		super(Explore, self).__init__(outcomes = ['succeeded', 'failed'], input_keys =['speed'])
		
		self._action_topic = '/move_base'
		self._move_client = ProxyActionClient({self._action_topic: MoveBaseAction})
		self._dynrec = Client("/vehicle_controller", timeout = 10)
		self._defaultspeed = 0.1
		self._succeeded = False
		self._failed = False

	def execute(self, userdata):

		if self._move_client.has_result(self._action_topic):
			result = self._move_client.get_result(self._action_topic)
			self._dynrec.update_configuration({'speed':self._defaultspeed})	
			if result.result == 1:
				self._reached = True
				Logger.loginfo('Exploration succeeded')
				return 'succeeded'
			else:
				self._failed = True
				Logger.logwarn('Exploration failed!')
				return 'failed'
		

	def on_enter(self, userdata):
			
		speedValue = self._dynrec.get_configuration(timeout = 0.5)
		if speedValue is not None:
			self._defaultspeed = speedValue['speed']	
			
		self._dynrec.update_configuration({'speed':userdata.speed})		
		self._succeeded = False
		self._failed = False
		
		action_goal = MoveBaseGoal()
		action_goal.exploration = True
		action_goal.speed = userdata.speed

		if action_goal.target_pose.header.frame_id == "":
			action_goal.target_pose.header.frame_id = "world"

		try:
			if self._move_client.is_active(self._action_topic):
				self._move_client.cancel(self._action_topic)
			self._move_client.send_goal(self._action_topic, action_goal)
		except Exception as e:
			Logger.logwarn('Failed to start Exploration' % {
				'err': str(e),
				'x': userdata.waypoint.pose.position.x,
				'y': userdata.waypoint.pose.position.y
			})
			self._failed = True
		
		


	def on_exit(self, userdata):
		self._move_client.cancel(self._action_topic)


	def on_start(self):
		pass

	def on_stop(self):
		pass
		
