#!/usr/bin/env python
import rospy
import smach

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller, ProxySubscriberCached


from task_msgs.srv import GetTask, GetTaskRequest
from sar_msgs.msg import SarTaskTypes

from task_msgs.msg import TaskState, Task


class Wait_getCloserVictim(EventState):
	'''
	Example for a state to demonstrate which functionality is available for state implementation.
	This example lets the behavior wait until the given target_time has passed since the behavior has been started.

	-- target_time 	float 	Time which needs to have passed since the behavior started.

	<= continue 			Given time has passed.
	<= failed 				Example for a failure outcome.

	'''

	def __init__(self):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(Wait_getCloserVictim, self).__init__(outcomes = ['waiting','restart','preempted'], input_keys = ['task_details_task_id', 'params_distance'], output_keys = ['pose', 'params_distance'])
		
		self._allocated_task = '/taskallocation/allocatedTask'
		self._sub = ProxySubscriberCached({self._allocated_task: Task})
		
		
		self._topic_get_task = 'taskallocation/get_task'
		self.getTask = ProxyServiceCaller({self._topic_get_task: GetTask})


	def execute(self, userdata):

		if self.preempt_requested(): 
            		self.service_preempt()
            		return 'preempted'
        
       	 	request = GetTaskRequest()
		# before userdata.task_details_task_id
        	request.task_id = 'victim_0_autonomous'
        
        	try:
            		response = self.getTask.call(self._topic_get_task, request).task
        	except Exception, e:
            		Logger.loginfo('[behavior_get_closer] Could not get task:' + str(e))
            		rospy.sleep(0.5)
            		return 'waiting'
        
        	if not response.details.floatParams[SarTaskTypes.INDEX_VICTIM_DISTANCE] == userdata.params_distance:
            		userdata.pose.pose = response.pose
            		userdata.params_distance = response.details.floatParams[SarTaskTypes.INDEX_VICTIM_DISTANCE]
            		return 'restart'
        	else:
            		rospy.sleep(0.5)
            		return 'waiting'
	
		

	def on_enter(self, userdata):
		
		task_local = self._sub.get_last_msg(self._allocated_task)
		if len(task_local.details.floatParams) != 4:
			pass
		else:
			userdata.task_details_task_id = task_local.details.task_id
        		userdata.pose.pose = task_local.pose
			Logger.loginfo(str(task_local.details.floatParams[4]))
        		userdata.params_distance = task_local.details.floatParams[SarTaskTypes.INDEX_VICTIM_DISTANCE]
		

	
	def on_exit(self, userdata):
	

		pass # Nothing to do in this example.


	def on_start(self):
		pass


	def on_stop(self):
		

		pass # Nothing to do in this example.
		
