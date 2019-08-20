#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached, ProxyServiceCaller
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from hector_std_msgs.srv import GetPoseArrayService, GetPoseArrayServiceRequest
from std_msgs.msg import Empty



class GetPathFromServiceState(EventState):
	'''
	Imports a path from a topic and returns a list of PoseStamped

	-- pathTopic	String		String containing topic where path is imported from

	#> waypoints 		PoseStamped[]	List of waypoints imported

	<= succeeded 				Waypoints have been parsed successfully.

	'''

	def __init__(self, service_topic='/path_to_follow'):
		super(GetPathFromServiceState, self).__init__(outcomes = ['succeeded', 'failed'], output_keys = ['waypoints2'])
		
		self._serviceTopic = service_topic
		self._posearraysub = ProxyServiceCaller({self._serviceTopic: GetPoseArrayService})
		self._succeeded = False
		self._failed = False

		
        	

	def execute(self, userdata):		
		if self._failed == True:
			return 'failed'
		if self._succeeded == True:
			return 'succeeded' 



	def on_enter(self, userdata):
		self._succeeded = False
		self._failed = False
		self._emptymsg = GetPoseArrayServiceRequest()


		try:
			Logger.logwarn('1')
                        userdata.waypoints2 = self._posearraysub.call(self._serviceTopic, self._emptymsg)
			Logger.logwarn('2')
                        self._succeeded = True
		except Exception as e:
			Logger.logwarn('Failed to send execute trajectory for arm motion:\n%s' % str(e))
			self._failed = True
		
		

	def on_exit(self, userdata):
		pass

	def on_start(self):		
		pass

	def on_stop(self):
		pass
		
