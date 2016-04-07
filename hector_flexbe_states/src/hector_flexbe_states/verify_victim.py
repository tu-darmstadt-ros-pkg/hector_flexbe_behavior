#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger


class verify_victim(EventState):
	'''
	part of ExploreVictim state machine

	'''

	def __init__(self):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(verify_victim, self).__init__(outcomes = ['true','false'], input_keys = ['task_details_task_id'])

		self._topic_get_task = 'taskallocation/get_task'
		self.getTask = ProxyServiceCaller({self._topic_get_task: GetTask})

	def execute(self, userdata):
		request = GetTaskRequest()
        	request.task_id = userdata.task_details_task_id
        
        	try:
			response = self.getTask.call(self._topic_get_task, request) #.task
        	except Exception, e:
            		Logger.logwarn('[hector_behavior ] (explore victim - verify victim) Could not get task:' + str(e))
            		return 'false'
        
        	if response.details.floatParams[SarTaskTypes.INDEX_VICTIM_SUPPORT] >= 1:
            		return 'true'
       	 	else:
            		return 'false'
	
		

	def on_enter(self, userdata):
		pass

	
	def on_exit(self, userdata):
	

		pass # Nothing to do in this example.


	def on_start(self):
		pass


	def on_stop(self):
		

		pass # Nothing to do in this example.
		
