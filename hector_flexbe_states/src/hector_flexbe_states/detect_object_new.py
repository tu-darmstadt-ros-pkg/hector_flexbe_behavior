#!/usr/bin/env python
import rospy

from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxySubscriberCached
from hector_worldmodel_msgs.msg import Object
from geometry_msgs.msg import PoseStamped
from rospy import Time


class DetectObjectNew(EventState):
	'''
	Observe the update of objects in the worldmodel.
	># type	   string		object_type to be found
	># state   string		state the object should be in

	<= found 			Object was found

	#> pose    PoseStamped		Pose of the object, which was detected
	#> object_id  string		object_id of detected object

	'''

	def __init__(self):
		
		super(DetectObjectNew, self).__init__(outcomes = ['found'], input_keys =['type', 'state', 'pose'], output_keys = ['pose', 'object_id'])

		self._objectTopic = '/worldmodel/object'
		self._sub = ProxySubscriberCached({self._objectTopic: Object})


	def execute(self, userdata):
	

		current_obj = self._sub.get_last_msg(self._objectTopic)
		if current_obj:
			if current_obj.info.class_id == userdata.type and current_obj.state.state == userdata.state: 
				userdata.pose = PoseStamped()
				userdata.pose.pose = current_obj.pose.pose
				userdata.pose.header.stamp = Time.now()
				userdata.pose.header.frame_id = 'map'
				userdata.object_id = current_obj.info.object_id
				Logger.loginfo('detected %(x)s' % {
					'x': current_obj.info.object_id
				})
				return 'found'
			
		

	def on_enter(self, userdata):

		pass

	def on_exit(self, userdata):
	
		pass 


	def on_start(self):
		
		pass

	def on_stop(self):
	
		pass 
		
