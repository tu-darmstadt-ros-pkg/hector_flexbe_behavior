#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient, ProxyPublisher

from hector_move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionPath
from rospy import Time
from geometry_msgs.msg import PoseStamped


'''
Created on 15.06.2015

@author: Philipp Schillinger
'''

class DrivepathTest(EventState):
	'''
	Lets the robot move along a given path.

	># path		PoseStamped[]			Array of Positions of the robot.
	># speed	float				Speed of the robot

	<= reached 					Robot is now located at the specified waypoint.
	<= failed 					Failed to send a motion request to the action server.
	'''

	def __init__(self):
		'''
		Constructor
		'''
		super(DrivepathTest, self).__init__(outcomes=['reached', 'failed'])
		
		self._failed = False
		self._reached = False

		self._pathTopic = '/controller/path'
		self._pub = ProxyPublisher({self._pathTopic: MoveBaseActionPath})
		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		
		if self._failed:
			return 'failed'
		if self._reached:
			return 'reached'

		

			
	def on_enter(self, userdata):
		
		self._path = MoveBaseActionPath()
		self._point = PoseStamped()
		self._point.pose.orientation.w = 1
		self._point.pose.position.x = 1
		self._point.header.frame_id = 'map'
		self._path.goal.target_path.poses.append(self._point)
		self._point.pose.position.x = 2
		self._path.goal.target_path.poses.append(self._point)
		self._point.pose.position.x = 3
		self._path.goal.target_path.poses.append(self._point)
		Logger.loginfo('%(x).3f %(y).3f %(z).3f' % {'x': self._point.pose.orientation.x, 'y': self._point.pose.orientation.y, 'z': self._point.pose.orientation.z})
		self._path.header.frame_id = 'map'
		self._path.goal.target_path.header.frame_id = 'map'


		self._failed = False

		
		self._pub.publish(self._pathTopic, self._path)
		self._reached = True
		
			

	def on_stop(self):
			pass

	def on_exit(self, userdata):
		
			pass

	def on_pause(self):
		self._move_client.cancel(self._action_topic)

	def on_resume(self, userdata):
		self.on_enter(userdata)
