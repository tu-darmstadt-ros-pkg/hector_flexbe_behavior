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
		super(Wait_getCloserVictim, self).__init__(outcomes = ['waiting','restart','preempted'], input_keys = ['task_details_task_id', 'params_distance'], output_keys = ['pose_position_x','pose_position_y','pose_position_z','pose_orientation_x','pose_orientation_y','pose_orientation_z','pose_orientation_w', 'params_distance'])
		
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
            		userdata.pose_position_x = response.pose.position.x
			userdata.pose_position_y = response.pose.position.y
			userdata.pose_position_z = response.pose.position.z
			userdata.pose_orientation_x = response.pose.orientation.x
			userdata.pose_orientation_y = response.pose.orientation.y
			userdata.pose_orientation_z = response.pose.orientation.z
			userdata.pose_orientation_w = response.pose.orientation.w
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
        		userdata.pose_position_x = task_local.pose.position.x
			userdata.pose_position_y = task_local.pose.position.y
			userdata.pose_position_z = task_local.pose.position.z
			userdata.pose_orientation_x = task_local.pose.orientation.x
			userdata.pose_orientation_y = task_local.pose.orientation.y
			userdata.pose_orientation_z = task_local.pose.orientation.z
			userdata.pose_orientation_w = task_local.pose.orientation.w
			Logger.loginfo(str(task_local.details.floatParams[4]))
        		userdata.params_distance = task_local.details.floatParams[SarTaskTypes.INDEX_VICTIM_DISTANCE]
		

	
	def on_exit(self, userdata):
	

		pass # Nothing to do in this example.


	def on_start(self):
		pass


	def on_stop(self):
		

		pass # Nothing to do in this example.
		
