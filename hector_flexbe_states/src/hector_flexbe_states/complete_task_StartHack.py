#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher

from task_msgs.srv import SetTaskState, SetTaskStateRequest
from task_msgs.msg import TaskState


class complete_task_StartHack(EventState):
	'''
	Example for a state to demonstrate which functionality is available for state implementation.
	This example lets the behavior wait until the given target_time has passed since the behavior has been started.

	-- target_time 	float 	Time which needs to have passed since the behavior started.

	<= continue 			Given time has passed.
	<= failed 				Example for a failure outcome.

	'''

	def __init__(self):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(complete_task_StartHack, self).__init__(outcomes = ['error','done'], input_keys = ['task_details_task_id'])
		
		self._topic_set_task_state = 'taskallocation/set_task_state'
		self._pub = ProxyPublisher({self._topic_set_task_state: SetTaskState})

        	self._task_state_completed = TaskState()
        	self._task_state_completed.state = TaskState.COMPLETED

		
	def execute(self, userdata):

		request = SetTaskStateRequest()
        	request.task_id = userdata.task_details_task_id
        	request.new_state = self._task_state_completed
        
        	try:
			self._pub.publish(self._topic_set_task_state, request)
            		return 'done'
        	except Exception, e:
            		Logger.logwarn('Could not set task state:' + str(e))
            		return 'error'		

		

	def on_enter(self, userdata):
		pass

	
	def on_exit(self, userdata):
	

		pass # Nothing to do in this example.


	def on_start(self):
		pass


	def on_stop(self):
		

		pass # Nothing to do in this example.
		
