#!/usr/bin/env python
import rospy


from flexbe_core import EventState, Logger

# added
from flexbe_core.proxy import ProxySubscriberCached

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseActionResult
from hector_move_base_msgs.msg import MoveBaseActionExplore


class Wait_Exploration(EventState):
	'''
	Example for a state to demonstrate which functionality is available for state implementation.
	This example lets the behavior wait until the given target_time has passed since the behavior has been started.

	-- target_time 	float 	Time which needs to have passed since the behavior started.

	<= continue 			Given time has passed.
	<= failed 				Example for a failure outcome.

	'''

	def __init__(self):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(Wait_Exploration, self).__init__(outcomes = ['aborted','succeeded', 'waiting'], input_keys=['goalId'], output_keys=['goalId'])

		self._resultTopic = '/move_base/result'
		self._sub = ProxySubscriberCached({self._resultTopic: MoveBaseActionResult})
		

		self._restart = False
		self._waiting = False

		self._result = GoalStatus.PENDING
		


	def execute(self, userdata):
		# This method is called periodically while the state is active.
		# Main purpose is to check state conditions and trigger a corresponding outcome.
		# If no outcome is returned, the state will stay active.

		
		tmp = self._sub.get_last_msg(self._resultTopic)
		if not tmp:
			self._waiting = True
			return 'waiting'

		if tmp.status.goal_id.id == userdata.goalId:
			self._result = tmp.status.status
		else:
			self._result = GoalStatus.PENDING

		# Ensure that robots doesn't stop to explore
		if self._result == GoalStatus.SUCCEEDED:         		
			self._succeeded = True
			return 'succeeded'

		elif self._result == GoalStatus.ABORTED:        		
			self._aborted = True
			return 'aborted'
        	
		else:
			self._waiting = True
            		return 'waiting'

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
		
