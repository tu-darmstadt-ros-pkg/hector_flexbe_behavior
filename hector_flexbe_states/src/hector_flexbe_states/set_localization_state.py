#!/usr/bin/env python

import rospy
import math
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyPublisher

from geometry_msgs.msg import PoseWithCovarianceStamped


'''
Created on 25.06.2015

@author: Philipp Schillinger
'''

class SetLocalizationState(EventState):
	'''
	Sends an intial pose guess to localization and waits a short time for adjustments.

	># guess 	Pose 		Localization pose guess in world frame.

	<= done					Localization has been updated.

	'''

	def __init__(self):
		'''
		Constructor
		'''
		super(SetLocalizationState, self).__init__(outcomes=['done'],
													input_keys=['guess'])

		self._pose_topic = '/initialpose'

		self._pub = ProxyPublisher({self._pose_topic: PoseWithCovarianceStamped})

		self._start_time = None
		self._wait_time = rospy.Duration(2.0)
		

		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		if rospy.Time.now() - self._start_time > self._wait_time: 
			return 'done'


	def on_enter(self, userdata):
		p = userdata.guess
		Logger.loginfo('Localize around (%.1f, %.1f, %.1f)' % (p.position.x, p.position.y, p.position.z))
		
		msg = PoseWithCovarianceStamped()
		msg.header.frame_id = 'world'
		msg.pose.pose = p
		# from rviz, if required
		#msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]

		self._pub.publish(self._pose_topic, msg)
		
		self._start_time = rospy.Time.now()
