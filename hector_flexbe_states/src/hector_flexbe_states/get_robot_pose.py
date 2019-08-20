#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from geometry_msgs.msg import PoseStamped



class GetRobotPose(EventState):
	'''
	Retrieves the robots current pose and returns it


	#> pose 	PoseStamped	Position and Pose which has been marked

	<= succeeded 			Point has been marked successfully.

	'''

	def __init__(self):
		super(GetRobotPose, self).__init__(outcomes = ['succeeded'], output_keys = ['pose'])

		self._objectTopic = '/robot_pose'
		self._sub = ProxySubscriberCached({self._objectTopic: PoseStamped})

		self._succeeded = False
        	

	def execute(self, userdata):
		
		if self._succeeded == True:
			return 'succeeded' 
		

	def on_enter(self, userdata):

		userdata.pose = self._sub.get_last_msg(self._objectTopic)
		self._succeeded = True
		

	def on_exit(self, userdata):

		pass


	def on_start(self):
		
		pass

	def on_stop(self):

		pass
		
