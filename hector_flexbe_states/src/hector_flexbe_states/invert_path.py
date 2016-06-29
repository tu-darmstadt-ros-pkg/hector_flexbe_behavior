#!/usr/bin/env python
import math
import rospy
import tf
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient, ProxyPublisher

from hector_move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionPath
from rospy import Time

'''
Created on 15.06.2015

@author: Philipp Schillinger
'''

class InvertPath(EventState):
	'''
	Lets the robot move along a given path.

	># path		Path		Array of Positions of the robot.
	#> path		Path		Array of Positions of the robot.

	<= reached 					Robot is now located at the specified waypoint.
	<= failed 					Failed to send a motion request to the action server.
	'''

	def __init__(self):
		'''
		Constructor
		'''
		super(InvertPath, self).__init__(outcomes=['reached', 'failed'], input_keys=['path'], output_keys=['path'])
		
		self._failed = False
		self._reached = False

		#self._pub = ProxyPublisher({self._pathTopic: MoveBaseActionPath})
		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		
		if self._failed:
			return 'failed'
		if self._reached:
			return 'reached'

		

			
	def on_enter(self, userdata):
		
		#self._path = MoveBaseActionPath()
		for i in range(len(userdata.path.poses)):
			q = userdata.path.poses[i].pose.orientation
			rpy = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
			q = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2] + math.pi)
			userdata.path.poses[i].pose.orientation.x = q[0]
			userdata.path.poses[i].pose.orientation.y = q[1]
			userdata.path.poses[i].pose.orientation.z = q[2]
			userdata.path.poses[i].pose.orientation.w = q[3]
	
		userdata.path.poses = list(reversed(userdata.path.poses))
		#self._path.goal.target_path.poses = userdata.path.poses
		#self._path.goal.target_path.header.frame_id = 'map'

		self._failed = False

		self._reached = True	

	def on_stop(self):
		pass

	def on_exit(self, userdata):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		self.on_enter(userdata)
