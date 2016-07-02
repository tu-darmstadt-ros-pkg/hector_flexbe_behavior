#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyPublisher
from hector_worldmodel_msgs.msg import Object
from hector_worldmodel_msgs.msg import ObjectState
from hector_worldmodel_msgs.srv import SetObjectState, SetObjectStateRequest
from geometry_msgs.msg import PoseStamped
from rospy import Time
from flexbe_core.proxy import ProxyServiceCaller


class ConfirmVictim(EventState):
	'''
	Confirm current victim

	># victim 	string		object_id of detected victim

	<= confirmed 			Current victim was confirmed
	'''

	def __init__(self):
		
		super(ConfirmVictim, self).__init__(outcomes = ['confirmed'], input_keys = ['victim'])

		self._setVictimState = '/worldmodel/set_object_state'
		self._srvSet = ProxyServiceCaller({self._setVictimState: SetObjectState})


	def execute(self, userdata):

		return 'confirmed'
			
		

	def on_enter(self, userdata):

		state = ObjectState()
		state.state = -1		

		request = SetObjectStateRequest(userdata.victim, state)
		resp = self._srvSet.call(self._setVictimState, request)

		Logger.loginfo('%(x)s confirmed' % {
				'x': userdata.victim 
		})
		return 'succeeded'

	def on_exit(self, userdata):

		pass 


	def on_start(self):
		
		pass

	def on_stop(self):

		pass 
		
