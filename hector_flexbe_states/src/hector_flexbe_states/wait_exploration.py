#!/usr/bin/env python
import rospy


from flexbe_core import EventState, Logger

# added
from flexbe_core.proxy import ProxySubscriberCached

from actionlib_msgs.msg import GoalStatus
from hector_move_base_msgs.msg import MoveBaseActionResult
from hector_move_base_msgs.msg import MoveBaseActionExplore

#from sar_msgs.msg import VictimAnswer


class WaitExploration(EventState):
	'''
	Wait State for Exploration

	
	<= error 			Currently not triggered
	<= waiting 			If robot is still exploring this output is used
	<= pause			For manually pausing the Exploration Task

	'''

	def __init__(self):
		super(WaitExploration, self).__init__(outcomes = ['error', 'waiting', 'pause'])

		self._error = False
		self._waiting = False
		self._pause = False


	def execute(self, userdata):
		
		if self._waiting:
			return 'waiting'


	def on_enter(self, userdata):

		self._waiting = True


	def on_exit(self, userdata):
		pass 

	def on_start(self):
		pass


	def on_stop(self):
		pass 
		
