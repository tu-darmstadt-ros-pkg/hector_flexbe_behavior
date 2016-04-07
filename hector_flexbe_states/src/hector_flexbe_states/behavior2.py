#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger


class behavior2(EventState):
	'''
	Initialisierung
	'''

	def __init__(self):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(behavior2, self).__init__(outcomes = ['APPROACH_VICTIM','EXPLORE','EXPLORE_VICTIM_HYPOTHESIS','GO_TO','START','LOOK_AT','CANCEL'])


	def execute(self, userdata):
		return 'EXPLORE'
	
		

	def on_enter(self, userdata):

		pass
	
	def on_exit(self, userdata):
	

		pass # Nothing to do in this example.


	def on_start(self):
		pass


	def on_stop(self):
		

		pass # Nothing to do in this example.
		
