#!/usr/bin/env python

import rospy
import rospkg
import time
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient, ProxyServiceCaller, ProxyPublisher

from hector_std_msgs.srv import StringService
from std_msgs.msg import String


'''
Created on 15.06.2015

@author: Gabriel Huettenberger
'''

class Write2dMapState(EventState):
	'''
	Writes the 3d Map to a State.


	'''


	def __init__(self, writer_topic='/syscommand'):
		'''
		Constructor
		'''
		super(Write2dMapState, self).__init__(outcomes=['success', 'failed'])
		self._writer_topic = writer_topic
		self._publisher = ProxyPublisher({self._writer_topic: String})
                
                

		self._success = False
		self._failed = False
		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''

		if self._success:
			return 'success'
                if self._failed:
                        return 'failed'


			
	def on_enter(self, userdata):
		self._success = False

		try:
                        self._publisher.publish(self._writer_topic, "savegeotiff")
                        self._success = 'True'
		except Exception as e:
			Logger.logwarn('Failed to write map:\n%s' % str(e))
			self._failed = True
			

	def on_stop(self):
                pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		pass
