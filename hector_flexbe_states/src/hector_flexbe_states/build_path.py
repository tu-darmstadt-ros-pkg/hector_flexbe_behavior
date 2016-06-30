#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached
from geometry_msgs.msg import PoseStamped



class BuildPath(EventState):
	'''
	Retrieves the robots current pose and returns it


	># pose 	PoseStamped	Position and Pose which has been marked
	#> poses	PoseStamped[]   Positions of all marked points

	<= succeeded 			Point has been marked successfully.

	'''

	def __init__(self):
		super(BuildPath, self).__init__(outcomes = ['succeeded', 'next'], input_keys = ['pose', 'poses'], output_keys = ['poses'])

		

		self._succeeded = False
        	

	def execute(self, userdata):
		
		if self._succeeded == True:
			return 'succeeded' 
		

	def on_enter(self, userdata):

		userdata.poses.append(userdata.pose)
		self._succeeded = True
		

	def on_exit(self, userdata):

		pass


	def on_start(self):
		
		pass

	def on_stop(self):

		pass
		
