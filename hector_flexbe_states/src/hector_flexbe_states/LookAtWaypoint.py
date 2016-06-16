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

class LookAtWaypoint(EventState):
	'''
	Lets the robot move to a given waypoint.

	># waypoint		PoseStamped		Specifies the waypoint to which the robot should move.

	<= reached 						Robot is now located at the specified waypoint.
	<= failed 						Failed to send a motion request to the action server.

	'''

	def __init__(self):
		'''
		Constructor
		'''
		super(LookAtWaypoint, self).__init__(outcomes=['reached', 'failed'],
											input_keys=['waypoint'])
		
		self._action_topic = '/pan_tilt_sensor_head_joint_control/look_at'
		self._client = ProxyActionClient({self._action_topic: hector_perception_msgs.msg.LookAtAction})

		self._failed = False
		self._reached = False
		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		#if self._failed:
			#return 'failed'
		if self._reached:
			return 'reached'

		#if self._move_client.has_result(self._action_topic):
			#result = self._move_client.get_result(self._action_topic)
			#if result.result == 1:
				#self._reached = True
				#return 'reached'
			#else:
				#self._failed = True
				#Logger.logwarn('Failed to look at waypoint!')
				#return 'failed'

			
	def on_enter(self, userdata):
		self._failed = False
		self._reached = False

		action_goal = hector_perception_msgs.msg.LookAtGoal()
		action_goal.look_at_target.target_point.point = userdata.waypoint.pose.position
		if action_goal.look_at_target.target_point.header.frame_id == "":
			action_goal.look_at_target.target_point.header.frame_id = "map"

		try:
			self._client.send_goal(self._action_topic, action_goal)
		except Exception as e:
			Logger.logwarn('Failed to send LookAt request to waypoint (%(x).3f, %(y).3f):\n%(err)s' % {
				'err': str(e),
				'x': userdata.waypoint.pose.position.x,
				'y': userdata.waypoint.pose.position.y
			})
			self._failed = True
		
		Logger.loginfo('Looking at next waypoint')
		self._reached = True
			

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
