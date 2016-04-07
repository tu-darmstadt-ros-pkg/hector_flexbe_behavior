#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger

# same as confirm_victim.py except of discarding victim instead of confirming
class reached_victim_explore(EventState):
	'''
	reached_victim(self,parent,confirm=False) from statemachine hector_explore_victim
	-> discard victim
	'''

	def __init__(self):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(reached_victim_explore, self).__init__(outcomes = ['succeeded'], input_keys = ['task_details_task_id'])
		
        	self._topicVictimReached = 'victimReached'
        	self._pub = ProxyPublisher({self._topicVictimReached: VictimAnswer)

	def execute(self, userdata):
		answer = VictimAnswer()
        	answer.task_id = userdata.task_details_task_id;
        	answer.answer = VictimAnswer.DISCARD;
        	self._pub.publish(self._topicVictimReached, answer)
        
        	Logger.loginfo('Discarding Victim, id = ' + userdata.task_details_task_id)

        	return 'succeeded'
	
		

	def on_enter(self, userdata):
		pass

	
	def on_exit(self, userdata):
	

		pass # Nothing to do in this example.


	def on_start(self):
		pass


	def on_stop(self):
		

		pass # Nothing to do in this example.
		
