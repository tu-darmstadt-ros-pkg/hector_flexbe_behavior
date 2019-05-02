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
		self.followMission = False
		self.exploreMission = False
		self.combinedMission = False
		self.followLineMission = False
		self.failed = False

		
		
	def execute(self, userdata):

		if self.followLineMission:
			return 'followLineMission'
		if self.exploreMission:
			return 'exploreMission'
		if self.combinedMission:
			return 'combinedMission'
		if self.followMission:
			return 'followMission'
		if self.failed:
			return 'failed'

			
	def on_enter(self, userdata):
		
		self.followMission = False
		self.exploreMission = False
		self.combinedMission = False
		self.followLineMission = False
		self.failed = False
		if userdata.specialFunctionality == 'linefollowing':
			self.followLineMission = True
		elif userdata.waypointFollowing:
			self.followMission = True
		elif userdata.exploration:
			self.exploreMission = True
		else:
			self.failed = True
		
			

	def on_stop(self):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		pass

