#!/usr/bin/env python

import rospy
import math
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyServiceCaller

from geometry_msgs.msg import PoseStamped
from hector_worldmodel_msgs.srv import GetObjectModel


'''
Created on 15.06.2015

@author: Philipp Schillinger
'''

class GetObjectOfInterestWaypointState(EventState):
	'''
	Retrieves the waypoint of a given object of interest.

	-- object_name	string			Name of the object of interest.
	
	#> waypoint 	PoseStamped 	Waypoint of the given object.

	<= done 						Desired waypoint is available.
	<= unknown 						The specified object is not known.

	'''


	def __init__(self, object_name):
		'''
		Constructor
		'''
		super(GetObjectOfInterestWaypointState, self).__init__(outcomes=['done', 'unknown'],
											output_keys=['waypoint'])
				
		self._srv_topic = '/worldmodel/get_object_model'
		self._srv = ProxyServiceCaller({self._srv_topic: GetObjectModel})
		self._failed = False

		self._object_name = object_name
		self._objects = list()
		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		if self._failed:
			return 'unknown'

		for obj in self._objects:
			if obj.info.name == self._object_name:
				pose_stamped = PoseStamped()
				pose_stamped.header = obj.header
				pose_stamped.pose = obj.pose.pose
				userdata.waypoint = pose_stamped
				return 'done'

		Logger.logwarn('Unable to find object "%s" in known objects.' % self._object_name)
		return 'unknown'

	
	def on_enter(self, userdata):
		self._failed = False

		request = GetObjectModel._request_class()
		try:
			response = self._srv.call(self._srv_topic, request)
			self._objects = response.model.objects
		except rospy.ServiceException as e:
			Logger.logwarn("Unable to retrieve objects data:\n%s" % str(e))
			self._failed = True
			return

