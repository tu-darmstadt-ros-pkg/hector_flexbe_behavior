#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxySubscriberCached
from hector_worldmodel_msgs.msg import Object
from geometry_msgs.msg import PoseStamped
from rospy import Time


class DetectObject(EventState):
	'''
	Observe the update of objects (victims) in the worldmodel. When its state is 'active', we return it.

	># pose	   PoseStamped		Pose of object

	<= found 			Victim was found

	#> pose    PoseStamped		Pose of the object, which was detected
	#> victim  string		object_id of detected victim

	'''

	def __init__(self):
		
		super(DetectObject, self).__init__(outcomes = [ 'found'], input_keys =['pose'], output_keys = ['pose', 'victim'])

		self._objectTopic = '/worldmodel/object'
		self._sub = ProxySubscriberCached({self._objectTopic: Object})


	def execute(self, userdata):
	

		current_obj = self._sub.get_last_msg(self._objectTopic)
		if current_obj:
			if current_obj.info.class_id == 'victim' and current_obj.state.state == 2: 
				userdata.pose = PoseStamped()
				userdata.pose.pose = current_obj.pose.pose
				userdata.pose.header.stamp = Time.now()
				userdata.pose.header.frame_id = 'map'
				userdata.victim = current_obj.info.object_id
				Logger.loginfo(current_obj.info.object_id)
				return 'found'
			
		

	def on_enter(self, userdata):

		pass

	def on_exit(self, userdata):
	
		pass 


	def on_start(self):
		
		pass

	def on_stop(self):
	
		pass 
		
