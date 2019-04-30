#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

'''
Created on 24.04.2019

@author: Katrin Becker
'''

class SwitchLineFollowerDirectionState(EventState):
	'''
	The robot drives in a straight line until a line for line following is detected.

	-- desired_speed          float64              Driving speed.
        
    ># reverse                bool                 True if robot drives backwards

	<= finished 			                       Switched camera topic and driving direction

	'''

	def __init__(self, camera_topic_forwards='/front_rbgd_cam/color/image_raw', camera_topic_backwards='/back_rbgd_cam/color/image_raw'):
		'''
		Constructor
		'''
		super(SwitchLineFollowerDirectionState, self).__init__(outcomes=['finished'],input_keys=['reverse', 'camera_topic'], output_keys=['reverse', 'camera_topic'])
		
		self._finished = False

		self._camera_topic_forwards = camera_topic_forwards
		self._camera_topic_backwards = camera_topic_backwards


		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		
		if self._finished:
			return 'finished'
 		
		

			
	def on_enter(self, userdata):
		self._finished = False        

		userdata.reverse = not userdata.reverse

		if userdata.reverse:
			userdata.camera_topic = self._camera_topic_backwards
		else:
			userdata.camera_topic = self._camera_topic_forwards

		Logger.loginfo('Swichted driving direction. New camera topic: %s' % userdata.camera_topic)


		self._finished = True

		
			

	def on_stop(self):
		pass
            
	def on_pause(self):
		pass
            
	def on_resume(self, userdata):
		pass
