#!/usr/bin/env python
import rospy


from flexbe_core import EventState, Logger

# added
from flexbe_core.proxy import ProxySubscriberCached

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseActionResult
from hector_move_base_msgs.msg import MoveBaseActionExplore

from sar_msgs.msg import VictimAnswer


class Object_Found_Exploration(EventState):
	'''
	Example for a state to demonstrate which functionality is available for state implementation.
	This example lets the behavior wait until the given target_time has passed since the behavior has been started.

	-- target_time 	float 	Time which needs to have passed since the behavior started.

	<= continue 			Given time has passed.
	<= failed 				Example for a failure outcome.

	'''

	def __init__(self):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(Object_Found_Exploration, self).__init__(outcomes = ['unknown', 'qr_code', 'victim'], input_keys=['object'], output_keys=['pose'])


		self._error = False
		self._qr_code = False
		self._victim = False

		


	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.

		if self._object.answer == 1:
			return 'victim'
		elif self._object.answer == 2:
			return 'qr_code'
		else:
			return 'unknown'

	def on_enter(self, userdata):
		# This method is called when the state becomes active, i.e. a transition from another state to this one is taken.
		# It is primarily used to start actions which are associated with this state.

		pass


	def on_exit(self, userdata):
		# This method is called when an outcome is returned and another state gets active.
		# It can be used to stop possibly running processes started by on_enter.

		pass 

	def on_start(self):
		# This method is called when the behavior is started.
		# If possible, it is generally better to initialize used resources in the constructor
		# because if anything failed, the behavior would not even be started.

		pass


	def on_stop(self):
		# This method is called whenever the behavior stops execution, also if it is cancelled.
		# Use this event to clean up things like claimed resources.

		pass 
		
