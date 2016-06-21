#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyPublisher
from hector_worldmodel_msgs.msg import Object
from hector_worldmodel_msgs.msg import ObjectState
from hector_worldmodel_msgs.srv import SetObjectState
from geometry_msgs.msg import PoseStamped
from rospy import Time


class ConfirmVictim(EventState):
	'''
	Confirm current victim

	># victim 	string		object_id of detected victim

	<= confirmed 			Current victim was confirmed
	'''

	def __init__(self):
		
		super(ConfirmVictim, self).__init__(outcomes = ['confirmed'], input_keys = ['victim'])

		

		self.set_victim_state = rospy.ServiceProxy('/worldmodel/set_object_state', SetObjectState)


	def execute(self, userdata):

		return 'confirmed'
			
		

	def on_enter(self, userdata):

		state = ObjectState()
		state.state = -1		
		self.set_victim_state(userdata.victim, state)
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
		
