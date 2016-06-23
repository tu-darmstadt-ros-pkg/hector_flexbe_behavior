#!/usr/bin/env python
import rospy

from flexbe_core.proxy import ProxyActionClient

from flexbe_core import EventState, Logger
from rospy import Time
import time
from hector_move_base_msgs.msg import MoveBaseAction, MoveBaseGoal




class Explore(EventState):
	'''
	Starts the Exploration Task via /move_base

	
	<= succeeded			Exploration Task was successful
	<= failed 			Exploration Task failed

	'''

	def __init__(self):
		super(Explore, self).__init__(outcomes = ['succeeded', 'failed'])
		
		self._action_topic = '/move_base'
		self._move_client = ProxyActionClient({self._action_topic: MoveBaseAction})

		self._succeeded = False
		self._failed = False
		self._robot_speed = 0.2

	def execute(self, userdata):
		#time.sleep(10)
		if self._move_client.has_result(self._action_topic):
			result = self._move_client.get_result(self._action_topic)
			if result.result == 1:
				self._reached = True
				Logger.loginfo('Exploration succeeded')
				return 'succeeded'
			else:
				self._failed = True
				Logger.logwarn('Exploration failed!')
				return 'failed'
		

	def on_enter(self, userdata):

		self._succeeded = False
		self._failed = False
		
		action_goal = MoveBaseGoal()
		action_goal.exploration = True
		action_goal.speed = self._robot_speed
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
		pass


	def on_start(self):
		pass

	def on_stop(self):
		pass
		
