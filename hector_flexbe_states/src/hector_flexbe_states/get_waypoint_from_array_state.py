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

		
		
	def execute(self, userdata):
		return 'succeeded'
		

			
	def on_enter(self, userdata):
		if not userdata.waypoints:
			return 'empty'
		else:
			userdata.waypoint = userdata.waypoints[self._position]
			userdata.waypoints.pop(self._position)
			return 'succeeded'
			

	def on_stop(self):
		pass
            
	def on_pause(self):
		pass
            
	def on_resume(self, userdata):
		pass
