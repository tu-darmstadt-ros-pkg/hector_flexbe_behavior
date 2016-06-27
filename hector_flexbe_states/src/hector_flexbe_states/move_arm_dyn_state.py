#!/usr/bin/env python
import math
import rospy, tf
from copy import deepcopy
from geometry_msgs.msg import Quaternion
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient
from argo_move_group_msgs.msg import ArgoCombinedPlanAction, ArgoCombinedPlanGoal, ActionCodes, ErrorCodes, ObjectTypes

'''
Created on 26.10.2015

@author: Markus Sigg
'''

class MoveArmDynState(EventState):
	'''
	Lets the robot move its arm.

	># object_pose		PoseStamped		Pose of the object to observe.
	># object_type		string			Object type.
	># object_id		string			ID of the object.

	<= reached 						Target joint configuration has been reached.
	<= sampling_failed				Failed to find any valid and promising camera pose.
	<= planning_failed 				Failed to find a plan to move the arm to a desired camera pose.
	<= control_failed 				Failed to move the arm along the planned trajectory.

	'''


	def __init__(self):
		'''
		Constructor
		'''
		super(MoveArmDynState, self).__init__(outcomes=['reached', 'sampling_failed', 'planning_failed', 'control_failed'],
											input_keys=['object_pose', 'object_type', 'object_id'])

		self._action_topic = '/combined_planner'
		self._client = ProxyActionClient({self._action_topic: ArgoCombinedPlanAction})
		
		# rotations for different object types to get z-axis aligned with optical axis
		self._rot = tf.transformations.quaternion_about_axis(-0.5*math.pi, (0,1,0))

		self._sampling_failed = False
		self._planning_failed = False
		self._control_failed = False
		self._success = False


	def execute(self, userdata):
		'''
		Execute this state
		'''
		if self._sampling_failed:
			return 'sampling_failed'
		if self._planning_failed:
			return 'planning_failed'
		if self._control_failed:
			return 'control_failed'
		if self._success:
			return 'reached'

		if self._client.has_result(self._action_topic):
			result = self._client.get_result(self._action_topic)

			if result.success.val == ErrorCodes.SUCCESS:
				self._success = True
				return 'reached'
			elif result.success.val == ErrorCodes.SAMPLING_FAILED:
				Logger.logwarn('Sampling failed when attempting to move the arm')
				self._sampling_failed = True
				return 'sampling_failed'	
			elif result.success.val == ErrorCodes.PLANNING_FAILED:
				Logger.logwarn('Planning failed when attempting to move the arm')
				self._planning_failed = True
				return 'planning_failed'
			else:
				Logger.logwarn('Control failed when attempting to move the arm')
				self._control_failed = True
				return 'control_failed'



	def on_enter(self, userdata):
		self._sampling_failed = False
		self._planning_failed = False
		self._control_failed = False
		self._success = False

		action_goal = ArgoCombinedPlanGoal()
		action_goal.target = deepcopy(userdata.object_pose)
		
		type_map = {
			'dial_gauge': ObjectTypes.DIAL_GAUGE,
			'level_gauge': ObjectTypes.LEVEL_GAUGE,
			'valve': ObjectTypes.VALVE,
			'hotspot': ObjectTypes.HOTSPOT,
			'pipes' : ObjectTypes.PIPES,
			'victim' : ObjectTypes.VICTIM
		}
		object_type = type_map.get(userdata.object_type, ObjectTypes.UNKNOWN)
		
		q = [ action_goal.target.pose.orientation.x, action_goal.target.pose.orientation.y,
			  action_goal.target.pose.orientation.z, action_goal.target.pose.orientation.w ]
		q = tf.transformations.quaternion_multiply(q, self._rot)
		action_goal.target.pose.orientation = Quaternion(*q)
		
		action_goal.object_id.data = userdata.object_id
		action_goal.object_type.val = object_type
		action_goal.action_type.val = ActionCodes.SAMPLE_MOVE_ARM

		Logger.loginfo('Position arm to look at %s object' % str(userdata.object_type))
		
		try:
			self._client.send_goal(self._action_topic, action_goal)
		except Exception as e:
			Logger.logwarn('Failed to send move group goal for arm motion:\n%s' % str(e))
			self._control_failed = True
			

	def on_stop(self):
		try:
			if self._client.is_available(self._action_topic) \
			and not self._client.has_result(self._action_topic):
				self._client.cancel(self._action_topic)
		except:
			# client already closed
			pass


	def on_pause(self):
		self._client.cancel(self._action_topic)

	def on_resume(self, userdata):
		self.on_enter(userdata)
