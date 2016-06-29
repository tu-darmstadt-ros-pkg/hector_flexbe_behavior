#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxyServiceCaller
from ethzasl_icp_mapper.srv import SetMode, GetMode, SetModeRequest


from smach import CBState



class SetMappingState(EventState):
	'''
	Activate or deactivate mapping.

	-- active 	bool 	Mapping changed to active or inactive.

	<= succeeded 		Mapping changed.

	'''

	def __init__(self, active):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(SetMappingState, self).__init__(outcomes = ['succeeded'])
        	
		self._mappingTopicSet = '/mapper/set_mode'
		self._srvSet = ProxyServiceCaller({self._mappingTopicSet: SetMode})
		

		#self.set_mapper_mode = rospy.ServiceProxy('/mapper/set_mode', SetMode)
		self._switch = active

	def execute(self, userdata):
		
		return 'succeeded' 
		

	def on_enter(self, userdata):
		
		#map_state = userdata.switch
	  	#resp = self.set_mapper_mode(True, map_state, True)
		request = SetModeRequest(True,self._switch, True)
		#request.map = self._switch
		resp = self._srvSet.call(self._mappingTopicSet, request)
		
	  

	def on_exit(self, userdata):

		pass 


	def on_start(self):

		pass

	def on_stop(self):

		pass 
		
