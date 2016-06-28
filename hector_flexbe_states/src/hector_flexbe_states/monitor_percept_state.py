#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxySubscriberCached

from hector_worldmodel_msgs.msg import ObjectModel, ObjectState
from geometry_msgs.msg import PoseStamped

'''
Created on 06.03.2016

@author: Philipp Schillinger
'''

class MonitorPerceptState(EventState):
	'''
	Monitors the hector worldmodel for percepts
	and triggers a detection event if a percept has a high-enough support.

	-- required_support	float 		Support threshold which needs to be reached before an event is triggered.
	-- percept_class	string 		Class name of percepts to be monitored.
	-- track_percepts 	bool 		Defines if this state should track previously detected percepts.
									Recommended if it is not desired to react multiple times on the same percept.

	#> object_id 		string 		Identifier of the detected object.
	#> object_pose 		PoseStamped Pose of the detected object.
	#> object_data 		float[] 	Data associated with this object.

	<= detected 					Percept is detected.

	'''

	def __init__(self, required_support, percept_class, track_percepts = True):
		'''
		Constructor
		'''
		super(MonitorPerceptState, self).__init__(outcomes=['detected'],
												output_keys=['object_id', 'object_pose', 'object_data'])
		
		self._topic = '/worldmodel/objects'
		self._sub = ProxySubscriberCached({self._topic: ObjectModel})

		self._required_support = required_support
		self._percept_class = percept_class
		self._track_percepts = track_percepts
		self._percepts = list()
			
		
	def execute(self, userdata):
		if self._sub.has_msg(self._topic):
			msg = self._sub.get_last_msg(self._topic)

			objects = filter(lambda obj:
				#obj.state.state == ObjectState.ACTIVE \
				obj.info.class_id == self._percept_class \
				and obj.info.support >= self._required_support \
				and obj.info.object_id not in self._percepts,
				msg.objects
			)

			for obj in objects:
			
				if self._track_percepts:
					self._percepts.append(obj.info.object_id)

				(x,y,z) = (obj.pose.pose.position.x, obj.pose.pose.position.y, obj.pose.pose.position.z)
				Logger.loginfo('Detected %s percept with id: %s\nPosition: (%.1f, %.1f, %.1f)' % (self._percept_class, obj.info.object_id, x, y, z))

				userdata.object_id = obj.info.object_id
				userdata.object_pose = PoseStamped(header=obj.header, pose=obj.pose.pose)
				userdata.object_data = obj.info.data
				return 'detected'
