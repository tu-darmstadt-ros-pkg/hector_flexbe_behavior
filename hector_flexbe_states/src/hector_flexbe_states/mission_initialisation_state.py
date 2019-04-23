#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger
import dynamic_reconfigure.client

'''
Created on 09.04.2019

@author: Gabriel Huettenberger
'''

class MissionInitialisationState(EventState):
	'''
	Decides what part of the mission to execute next.

	'''

	def __init__(self):
		'''
		Constructor
		'''
		super(MissionInitialisationState, self).__init__(outcomes=['done', 'failed'], input_keys=['hazmatEnabled', 'traversabilityMap', 'roughTerrain'])

		client = dynamic_reconfigure.client.Client("vehicle_controller", timeout=10)

		
		
	def execute(self, userdata):

		continue

			
	def on_enter(self, userdata):
		
		if userdata.hazmatEnabled
			continue
		if userdata.traversabilityMap
			 client.update_configuration({"lethal_dist":x, "double_param":(1/(x+1)), "str_param":str(rospy.get_rostime()), "bool_param":b, "size":1})
		if userdata.roughTerrain
			 client.update_configuration({"int_param":x, "double_param":(1/(x+1)), "str_param":str(rospy.get_rostime()), "bool_param":b, "size":1})
		return 'done'
		
			

	def on_stop(self):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		pass

