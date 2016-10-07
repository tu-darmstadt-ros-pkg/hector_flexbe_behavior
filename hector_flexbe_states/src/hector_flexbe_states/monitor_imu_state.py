#!/usr/bin/env python

import rospy
import math
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxySubscriberCached

from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion


'''
Created on 17.06.2015

@author: Philipp Schillinger
'''

class MonitorIMUState(EventState):
	'''
	Monitors the IMU data for a certain absolute tilt threshold.

	-- operation 		int 		One of the class constants to select the comparation operation.

	># tilt_threshold	float		Absolute threshold when this state should return.
									Has to be positive.

	<= triggered					Threshold has been reached.

	'''

	GREATER = 1
	LESS 	= -1


	def __init__(self, operation):
		'''
		Constructor
		'''
		super(MonitorIMUState, self).__init__(outcomes=['triggered'],
											input_keys=['tilt_threshold'])

		self._imu_topic = '/imu/data'
		self._op = operation
                
		self._sub = ProxySubscriberCached({self._imu_topic: Imu})
		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		if self._sub.has_msg(self._imu_topic):
			msg = self._sub.get_last_msg(self._imu_topic)
			self._sub.remove_last_msg(self._imu_topic)
			q = msg.orientation
			rpy = euler_from_quaternion([q.x,q.y,q.z,q.w])
			pitch = rpy[1]
			if self._op * abs(pitch) > self._op * userdata.tilt_threshold:
				print 'Trigger with pitch: %.5f' % pitch
				return 'triggered'
