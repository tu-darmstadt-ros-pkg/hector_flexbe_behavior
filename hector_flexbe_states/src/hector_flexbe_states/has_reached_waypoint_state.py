#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxySubscriberCached

from move_base_lite_msgs.msg import MoveBaseAction, MoveBaseGoal, ErrorCodes, PlanPathOptions

from geometry_msgs.msg import PoseStamped, Twist
'''
Created on 15.06.2015

@author: Gabriel Huettenberger
'''

class HasReachedWaypointState(EventState):
	'''
	The robot drives in a straight line.

	-- threshold          float64              Maximum distance to waypoint to return reached.



	<= reached 						Robot drove for the specified duration.
	<= failed 						Received a new motion command while driving.

	'''

	def __init__(self, threshold=0.1, robot_pose_topic='/robot_pose'):
		'''
		Constructor
		'''
		super(HasReachedWaypointState, self).__init__(outcomes=['reached', 'failed'], input_keys=['waypoint'])
		
		self._threshold = threshold
		self._robot_pose_topic = robot_pose_topic
		self._sub = ProxySubscriberCached({self._robot_pose_topic: PoseStamped})
		self._failed = False
		self._reached = False
		self._currentPose = PoseStamped()


		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		if self._failed:
			return 'failed'
		if self._reached:
			return 'reached'
                x_dist = userdata.waypoint.pose.position.x - self._currentPose.pose.position.x
                y_dist = userdata.waypoint.pose.position.y - self._currentPose.pose.position.y
                dist = ((x_dist**2) + (y_dist**2))**0.5
                Logger.loginfo(dist)

                if (dist < self._threshold):
                    self._reached = True
     		self._currentPose = self._sub.get_last_msg(self._robot_pose_topic)

		

			
	def on_enter(self, userdata):
		self._failed = False
		self._reached = False
		self._currentPose = self._sub.get_last_msg(self._robot_pose_topic)
		
			

	def on_stop(self):
                pass
            
	def on_pause(self):
                pass
            
	def on_resume(self, userdata):
                pass