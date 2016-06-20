#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger


class ErrorExploration(EventState):
	'''		
	Error Handling for Exploration Behavior	

	<= restart			Restarts Exploration

	'''

	def __init__(self):

		super(ErrorExploration, self).__init__(outcomes = ['restart'])

		self._restart = False

	def execute(self, userdata):
		
		if self._restart == True:
			return 'restart'

	def on_enter(self, userdata):

		self._restart = True
		Logger.loginfo('Error occurred, restarting')

	def on_exit(self, userdata):

		pass 

	def on_start(self):

		pass

	def on_stop(self):

		pass 
