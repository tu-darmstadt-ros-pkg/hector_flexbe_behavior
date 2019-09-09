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

class AddWaypointToArrayState(EventState):
	'''
	The robot drives in a straight line.

	-- threshold          float64              Maximum distance to waypoint to return reached.



	<= reached 						Robot drove for the specified duration.
	<= failed 						Received a new motion command while driving.

	'''

	def __init__(self):
		'''
		Constructor
		'''
		super(AddWaypointToArrayState, self).__init__(outcomes=['succeeded', 'failed'], input_keys=['waypoint', 'waypoints'], output_keys=['waypoints'])



		self._succeeded = False
		self._failed = False


		
		
	def execute(self, userdata):
		if self._succeeded:
			return 'succeeded'
		elif self._failed:
			return 'failed'
		

			
	def on_enter(self, userdata):
		
		self._succeeded = False
		self._failed = False

		if userdata.waypoint is None:
			self._failed = True
		else:
			userdata.waypoints.value.poses.append(userdata.waypoint.pose)
			self._succeeded = True
			

	def on_stop(self):
		pass
            
	def on_pause(self):
		pass
            
	def on_resume(self, userdata):
		pass
