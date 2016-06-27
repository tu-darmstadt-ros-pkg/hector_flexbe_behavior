#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxyServiceCaller
from ethzasl_icp_mapper.srv import SetMode, GetMode


from smach import CBState



class SwitchMapping(EventState):
	'''
	Switches mapping dependent on the switch input

	<= succeeded 			Mapping has been switched

	># switch	boolean		True to activate mapping

	'''

	def __init__(self):
		super(SwitchMapping, self).__init__(outcomes = ['succeeded'], input_keys = ['switch'])

        	self.set_mapper_mode = rospy.ServiceProxy('/mapper/set_mode', SetMode)

		self._succeeded = False

	def execute(self, userdata):

		if self._succeeded = True		
			return 'succeeded'
		

	def on_enter(self, userdata):
	
		map_state = userdata.switch
	  	resp = self.set_mapper_mode(True, map_state, True)
		self._succeeded = True
      		
	def on_exit(self, userdata):
		
		pass


	def on_start(self):

		pass

	def on_stop(self):
		
		pass 
		
