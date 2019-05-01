#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger
'''
Created on 09.04.2019

@author: Gabriel Huettenberger
'''

class MissionDecisionState(EventState):
	'''
	Decides what part of the mission to execute next.

	'''

	def __init__(self):
		'''
		Constructor
		'''
		super(MissionDecisionState, self).__init__(outcomes=['followMission', 'exploreMission', 'combinedMission', 'followLineMission', 'failed'], input_keys=['exploration', 'waypointFollowing', 'specialFunctionality'])

		
		
	def execute(self, userdata):

		pass

			
	def on_enter(self, userdata):
		
		if userdata.specialFunctionality == 'linefollowing':
			return 'followLineMission'
		if userdata.exploration and userdata.waypointFollowing:
			return 'combinedMission'
		elif userdata.waypointFollowing:
			return 'followMission'
		elif userdata.exploration:
			return 'exploreMission'
		else:
			return 'failed'
		
			

	def on_stop(self):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		pass

