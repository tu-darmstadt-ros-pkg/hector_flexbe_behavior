#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher


from sar_msgs.msg import VictimAnswer, SarTaskTypes


class confirm_victim(EventState):
	'''
	Example for a state to demonstrate which functionality is available for state implementation.
	This example lets the behavior wait until the given target_time has passed since the behavior has been started.

	-- target_time 	float 	Time which needs to have passed since the behavior started.

	<= continue 			Given time has passed.
	<= failed 				Example for a failure outcome.

	'''

	def __init__(self):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(confirm_victim, self).__init__(outcomes = ['succeeded'], input_keys = ['task_details_task_id'])
		
        	self._topicVictimReached = 'victimReached'
		self._pub = ProxyPublisher({self._topicVictimReached: VictimAnswer})


	def execute(self, userdata):
		answer = VictimAnswer()
        	answer.task_id = userdata.task_details_task_id;
        	answer.answer = VictimAnswer.CONFIRM
        	self._pub.publish(self._topicVictimReached, answer)
        
        	Logger.loginfo('Victim Found, id = ' + str(userdata.task_details_task_id))
		
        	return 'succeeded'
		

	def on_enter(self, userdata):
		pass
	
	
	def on_exit(self, userdata):
	

		pass # Nothing to do in this example.


	def on_start(self):
		pass


	def on_stop(self):
		

		pass # Nothing to do in this example.
		
