#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxySubscriberCached

from move_base_lite_msgs.msg import MoveBaseAction, MoveBaseGoal, ErrorCodes, PlanPathOptions

from geometry_msgs.msg import PoseStamped, Twist
'''
Created on 15.06.2015

@author: Gabriel Huettenberger
'''

class GetWaypointFromArrayState(EventState):
	'''
	The robot drives in a straight line.

	-- threshold          float64              Maximum distance to waypoint to return reached.



	<= reached 						Robot drove for the specified duration.
	<= failed 						Received a new motion command while driving.

	'''

	def __init__(self, position=0):
		'''
		Constructor
		'''
		super(GetWaypointFromArrayState, self).__init__(outcomes=['succeeded', 'empty'], input_keys=['waypoints'], output_keys=['waypoint', 'waypoints'])

		self._position = position

		self._succeeded = False
		self._empty = False


		
		
	def execute(self, userdata):
		if self._succeeded:
			return 'succeeded'
		elif self._empty:
			return 'empty'
		

			
	def on_enter(self, userdata):
		
		self._succeeded = False
		self._empty = False

		temp_waypoint = PoseStamped()
		temp_waypoint.header = userdata.waypoints.value.header

		if not userdata.waypoints.value.poses:
			self._empty = True
		else:
			temp_waypoint.pose = userdata.waypoints.value.poses[self._position]
			userdata.waypoints.value.poses.pop(self._position)
			userdata.waypoint = temp_waypoint
			self._succeeded = True
			

	def on_stop(self):
		pass
            
	def on_pause(self):
		pass
            
	def on_resume(self, userdata):
		pass
