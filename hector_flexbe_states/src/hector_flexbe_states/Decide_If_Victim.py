#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyPublisher
from hector_worldmodel_msgs.msg import Object
from hector_worldmodel_msgs.msg import ObjectState
from hector_worldmodel_msgs.srv import SetObjectState
from geometry_msgs.msg import PoseStamped
from rospy import Time


class DecideIfVictim(EventState):
	'''
	State for manual confirmation or discard of current victim. Additionally we can retry to drive to it.

	<= confirm			confirm victim
	<= discard 			discard victim
	<= retry			retry to drive to the victim
	'''

	def __init__(self):
		
		super(DecideIfVictim, self).__init__(outcomes = ['confirm', 'discard', 'retry'])

		self.set_victim_state = rospy.ServiceProxy('/worldmodel/set_object_state', SetObjectState)


	def execute(self, userdata):
		
		pass
			

	def on_enter(self, userdata):
		
		pass

	def on_exit(self, userdata):

		pass 


	def on_start(self):
		pass

	def on_stop(self):
		
		pass 
		
