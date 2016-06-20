#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyPublisher
from hector_worldmodel_msgs.msg import Object
from hector_worldmodel_msgs.msg import ObjectState
from hector_worldmodel_msgs.srv import SetObjectState
from geometry_msgs.msg import PoseStamped
from rospy import Time


class DiscardVictim(EventState):
	'''
	Discard current victim

	># victim	string			object_id of current victim

	<= discarded 				Current victim was discarded.

	'''

	def __init__(self):
		
		super(DiscardVictim, self).__init__(outcomes = ['discarded'], input_keys = ['victim'])


		self.set_victim_state = rospy.ServiceProxy('/worldmodel/set_object_state', SetObjectState)


	def execute(self, userdata):

		return 'discarded'
			
		

	def on_enter(self, userdata):

		state = ObjectState()
		state.state = -2		
		self.set_victim_state(userdata.victim, state)
		return 'discarded'

	def on_exit(self, userdata):

		pass 


	def on_start(self):
		
		pass

	def on_stop(self):

		pass 
		
