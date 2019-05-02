#!/usr/bin/env python

import rospy
import rospkg
import time
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient, ProxyServiceCaller

from hector_std_msgs.srv import StringService


'''
Created on 15.06.2015

@author: Gabriel Huettenberger
'''

class Write3dMapState(EventState):
	'''
	Writes the 3d Map to a State.


	'''


	def __init__(self, service_topic='/worldmodel_main/save_map', save_path='/octomaps/'):
		'''
		Constructor
		'''
		super(Write3dMapState, self).__init__(outcomes=['success', 'failed'])
		self._service_topic = service_topic
		self._service_client = ProxyServiceCaller({self._service_topic: StringService})
		r = rospkg.RosPack()
		self._save_path = r.get_path('robot_onboard_logging')+save_path
                
                

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
		timestr = time.strftime("%Y-%m-%d_%H-%M-%S")
		self._service_msg = self._save_path + timestr + ".bt"

		try:
                        self._service_client.call(self._service_topic, self._service_msg)
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
