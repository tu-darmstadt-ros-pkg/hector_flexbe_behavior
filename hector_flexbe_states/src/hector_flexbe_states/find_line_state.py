#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyPublisher

from geometry_msgs.msg import Twist

'''
Created on 24.04.2019

@author: Katrin Becker
'''

class FindLineState(EventState):
	'''
	The robot drives in a straight line until a line for line following is detected.

	-- speed          float64              Driving speed.
    -- distance       float64              Distance to drive.
        
    ># reverse                bool                 True if robot drives backwards

	<= reached 			      Robot drove the specified distance.

	'''

	def __init__(self, speed=0.2, distance=1, velocity_topic='/cmd_vel'):
		'''
		Constructor
		'''
		super(FindLineState, self).__init__(outcomes=['reached'],input_keys=['reverse'])
		
		self._velocity_topic = velocity_topic
		self._pub = ProxyPublisher({self._velocity_topic: Twist})
		self._reached = False
		self._speed = speed
		self._distance = distance
		self._msg = Twist()
		self._start_time = rospy.Time.now()
		


		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		
		if self._reached:
			return 'reached'
		temp_time = rospy.get_rostime() - self._start_time;

		duration = self._distance / self._speed;

 		if (temp_time.to_sec() > duration):
			self._start_time = rospy.get_rostime()
			self._pub.publish(self._velocity_topic, Twist())
			self._reached = True
		

			
	def on_enter(self, userdata):
		self._reached = False
		self._start_time = rospy.Time.now()
		self._msg.linear.x = self._speed
		if (userdata.reverse):
				self._msg.linear.x = self._msg.linear.x * -1
		self._pub.publish(self._velocity_topic, self._msg)
		
			

	def on_stop(self):
		pass
            
	def on_pause(self):
		pass
            
	def on_resume(self, userdata):
		pass
