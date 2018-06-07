#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyPublisher

from move_base_lite_msgs.msg import MoveBaseAction, MoveBaseGoal, ErrorCodes, PlanPathOptions

from geometry_msgs.msg import PoseStamped, Twist
'''
Created on 15.06.2015

@author: Gabriel Huettenberger
'''

class DriveStraightState(EventState):
	'''
	The robot drives in a straight line.

	-- desired_speed          float64              Driving speed.



	<= reached 						Robot drove for the specified duration.
	<= failed 						Received a new motion command while driving.

	'''

	def __init__(self, desired_speed=0.2, duration=10, reverse=False, velocity_topic='/cmd_vel'):
		'''
		Constructor
		'''
		super(DriveStraightState, self).__init__(outcomes=['reached', 'failed'])
		
		self._velocity_topic = velocity_topic
		self._pub = ProxyPublisher({self._velocity_topic: Twist})
		self._failed = False
		self._reached = False
		self._desired_speed = desired_speed
		self._msg = Twist()
		self._start_time = rospy.Time.now()
		self._duration = duration
		self._reverse = reverse


		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		if self._failed:
			return 'failed'
		if self._reached:
			return 'reached'
                temp_time = rospy.get_rostime() - self._start_time;
 		if (temp_time.to_sec() > self._duration):
			self._start_time = rospy.get_rostime()
                        self._pub.publish(self._velocity_topic, Twist())
                        self._reached = True
		

			
	def on_enter(self, userdata):
		self._failed = False
		self._reached = False
		self._start_time = rospy.Time.now()
                self._msg.linear.x = self._desired_speed
		if (self._reverse):
                    self._msg.linear.x = self._msg.linear.x * -1
		self._pub.publish(self._velocity_topic, self._msg)
		
			

	def on_stop(self):
                pass
            
	def on_pause(self):
                pass
            
	def on_resume(self, userdata):
                pass