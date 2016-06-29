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

		self._marker_topic = '/debug/path'

		self._pathTopic = '/controller/path'
		self._pub = ProxyPublisher({self._pathTopic: MoveBaseActionPath,
									self._marker_topic: MarkerArray})
		
		
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
		#userdata.path.poses = userdata.path.poses.reverse()
		for i in range(len(userdata.path.poses)):
			#userdata.path.poses[i].pose.position.x = -userdata.path.poses[i].pose.position.x
			#userdata.path.poses[i].pose.position.y = -userdata.path.poses[i].pose.position.y
			#userdata.path.poses[i].pose.position.z = -userdata.path.poses[i].pose.position.z
			# userdata.path.poses[i].pose.orientation.x = 0
			# userdata.path.poses[i].pose.orientation.y = 0
			# userdata.path.poses[i].pose.orientation.z = 0
			# userdata.path.poses[i].pose.orientation.w = 1
			q = userdata.path.poses[i].pose.orientation
			rpy = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
			q = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2] + math.pi)
			userdata.path.poses[i].pose.orientation.x = q[0]
			userdata.path.poses[i].pose.orientation.y = q[1]
			userdata.path.poses[i].pose.orientation.z = q[2]
			userdata.path.poses[i].pose.orientation.w = q[3]
			marker = Marker(type=Marker.ARROW)
			marker.header = self._path.goal.target_path.header
			marker.pose = userdata.path.poses[i].pose
			marker.scale.x = 0.2
			marker.scale.y = 0.02
			marker.scale.z = 0.02
			marker.color.b = 1.0
			marker.color.a = 0.5
			marker.id = i
			ma.markers.append(marker)
		userdata.path.poses = list(reversed(userdata.path.poses))
		self._path.goal.target_path.poses = userdata.path.poses
		self._path.goal.target_path.header.frame_id = 'map'

		self._failed = False

		
		self._pub.publish(self._pathTopic, self._path)
		self._pub.publish(self._marker_topic, ma)
		self._reached = True
		
			

	def on_stop(self):
			pass

	def on_exit(self, userdata):
		
			pass

	def on_pause(self):
		self._move_client.cancel(self._action_topic)

	def on_resume(self, userdata):
		self.on_enter(userdata)
