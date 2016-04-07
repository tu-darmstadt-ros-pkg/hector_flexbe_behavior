#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from std_msgs.msg import Empty, String


class Cancel(EventState):
	'''
	Example for a state to demonstrate which functionality is available for state implementation.
	This example lets the behavior wait until the given target_time has passed since the behavior has been started.

	-- target_time 	float 	Time which needs to have passed since the behavior started.

	<= continue 			Given time has passed.
	<= failed 				Example for a failure outcome.

	'''

	def __init__(self):
		# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.
		super(Cancel, self).__init__(outcomes = ['done'])

        	self._topic_cancel = '/move_base/cancel'
		self._pub1 = ProxyPublisher({self._topic_cancel: Empty)
        	self._proxy_publisher.createPublisher(self._topic_cancel, Empty)
        	self._topic_camera_pattern = '/camera/pattern'
        	# old: self._proxy_publisher.createPublisher(self._topic_camera_pattern, String, _latch=True)
		self._pub2 = ProxyPublisher({self._topic_camera_pattern: String, _latch=True)
		

	def execute(self, userdata):

		self._pub1.publish(self._topic_cancel, Empty())
		self._pub2.publish(self._topic_camera_pattern, '')
        
        return 'done'
		return 'done'
	
		

	def on_enter(self, userdata):
		pass

	
	def on_exit(self, userdata):
	

		pass # Nothing to do in this example.


	def on_start(self):
		pass


	def on_stop(self):
		

		pass # Nothing to do in this example.
		
