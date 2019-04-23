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
		super(MissionDecisionState, self).__init__(outcomes=['followMission', 'exploreMission', 'combinedMission', 'failed'], input_keys=['exploration', 'waypointFollowing'])

		
		
	def execute(self, userdata):

		continue

			
	def on_enter(self, userdata):
		
		if userdata.followMission and userdata.waypointFollowing
			return 'combinedMission'
		else if userdata.followMission
			return 'followMission'
		else if userdata.exploreMission
			return 'exploreMission'
		else
			return 'failed'
		
			

	def on_stop(self):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		pass

