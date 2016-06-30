#!/usr/bin/env python
import math
import rospy
import tf
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient, ProxyPublisher

from hector_move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionPath
from rospy import Time

from visualization_msgs.msg import MarkerArray, Marker


'''
Created on 15.06.2015

@author: Philipp Schillinger
'''

class MoveAlongPath(EventState):
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
		super(MoveAlongPath, self).__init__(outcomes=['reached', 'failed'], input_keys=['path','speed'])
		
		self._failed = False
		self._reached = False
	
		self._pathTopic = '/controller/path'
		self._marker_topic = '/debug/path'

		self._pub = ProxyPublisher({self._pathTopic: MoveBaseActionPath, self._marker_topic: MarkerArray})
		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		
		if self._failed:
			return 'failed'
		if self._reached:
			return 'reached'

			
	def on_enter(self, userdata):

		ma = MarkerArray()
		self._path = MoveBaseActionPath()
				
		for i in range(len(userdata.path.poses)):
			
			marker = Marker(type=Marker.ARROW)
			marker.header = userdata.path.header
			marker.pose = userdata.path.poses[i].pose
			marker.scale.x = 0.2
			marker.scale.y = 0.02
			marker.scale.z = 0.02
			marker.color.b = 1.0
			marker.color.r = 0.9 - 0.7 * i / len(userdata.path.poses)
			marker.color.g = 0.9 - 0.7 * i / len(userdata.path.poses)
			marker.color.a = 0.8 - 0.5 * i / len(userdata.path.poses)
			marker.id = i
			ma.markers.append(marker)

		self._failed = False

		self._path.goal.target_path.poses = userdata.path.poses
		self._path.goal.target_path.header.frame_id = 'map'
		
		self._pub.publish(self._pathTopic, self._path)
		self._pub.publish(self._marker_topic, ma)
		self._reached = True
		
			

	def on_stop(self):
		pass

	def on_exit(self, userdata):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		self.on_enter(userdata)
