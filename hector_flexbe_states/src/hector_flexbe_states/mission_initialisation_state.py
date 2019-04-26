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

		self.vehicle_controller_client = dynamic_reconfigure.client.Client("vehicle_controller", timeout=10)
		self.move_base_client = dynamic_reconfigure.client.Client("move_base_lite_node", timeout=10)

		
		
	def execute(self, userdata):

		return 'done'

			
	def on_enter(self, userdata):
		
		if userdata.hazmatEnabled:
			pass
		if userdata.traversabilityMap:
			self.move_base_client.update_configuration({"lethal_dist":2})
		else:
			self.move_base_client.update_configuration({"lethal_dist":0.5})		
		if userdata.roughTerrain:
			self.vehicle_controller_client.update_configuration({"angle_p_gain":3})
		else:
			self.vehicle_controller_client.update_configuration({"angle_p_gain":8})
		
			

	def on_stop(self):
		pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		pass

