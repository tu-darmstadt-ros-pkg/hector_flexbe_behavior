#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxyServiceCaller



from smach import CBState



class SwitchMapping(EventState):
	'''
	Switches mapping dependent on the switch input

	<= succeeded 			Mapping has been switched

	># switch	boolean		True to activate mapping

	'''

	def __init__(self, enable=True):
		super(SwitchMapping, self).__init__(outcomes = ['succeeded'])
		self._enable = enable
		self._enable_mapping = None
		try:
			rospy.wait_for_service('/enable_map_update', timeout=2)
        		self._enable_mapping = rospy.ServiceProxy('/enable_map_update', bool)
		except:
			Logger.logwarn('Enable map service not available')


		self._succeeded = False

	def execute(self, userdata):

		if self._succeeded == True:		
			return 'succeeded'
		

	def on_enter(self, userdata):
	
		if self._enable_mapping == None:
			Logger.logwarn('Enable map service not available')
		else:
	  		resp = self._enable_mapping(self._enable)
		self._succeeded = True
      		
	def on_exit(self, userdata):
		
		pass


	def on_start(self):

		pass

	def on_stop(self):
		
		pass 
		
