#!/usr/bin/env python

import rospy
import math
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyPublisher

from geometry_msgs.msg import Twist


'''
Created on 24.06.2015

@author: Philipp Schillinger
'''

class DrivePredefinedState(EventState):
	'''
	Commands a robot velocity for a certain time.

	-- duration 	float 	Duration in seconds of driving.

	># speed 		float 	Speed (m/s) with which the robot should drive.
							Pass a negative value for driving backwards.

	<= done					Time has elapsed.

	'''

	def __init__(self, duration):
		'''
		Constructor
		'''
		super(DrivePredefinedState, self).__init__(outcomes=['done'],
													input_keys=['speed'])

		self._vel_topic = '/cmd_vel'

		self._pub = ProxyPublisher({self._vel_topic: Twist})

		self._starting_time = None
		self._duration = rospy.Duration(duration)
		self._cmd_msg = Twist()
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		if rospy.Time.now() - self._starting_time >= self._duration:
			return 'done'

		self._pub.publish(self._vel_topic, self._cmd_msg)


	def on_enter(self, userdata):
		self._starting_time = rospy.Time.now()

		self._cmd_msg.linear.x = userdata.speed


	def on_exit(self, userdata):
		# always stop when leaving this state
		self._pub.publish(self._vel_topic, Twist())

	def on_stop(self):
		self._pub.publish(self._vel_topic, Twist())

	def on_pause(self):
		self._pub.publish(self._vel_topic, Twist())
