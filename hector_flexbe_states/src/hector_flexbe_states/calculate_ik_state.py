#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyServiceCaller
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import MoveItErrorCodes

'''
Created on 15.06.2015

@author: Philipp Schillinger
'''

class CalculateIKState(EventState):
	'''
	Calculates a joint target from the given endeffector pose.

	-- move_group 		string 			Name of the move group to be used for planning.
	-- ignore_collisions bool 			Ignore collision, use carefully.

	># eef_pose			PoseStamped		Target pose of the endeffector.

	#> joint_config 	float[] 		Calculated joint target.

	<= planned 							Found a plan to the target.
	<= failed 							Failed to find a plan to the given target.

	'''


	def __init__(self, move_group, ignore_collisions = False):
		'''
		Constructor
		'''
		super(CalculateIKState, self).__init__(outcomes=['planned', 'failed'],
											input_keys=['eef_pose'],
											output_keys=['joint_config'])
		
		self._topic = '/compute_ik'
		self._srv = ProxyServiceCaller({self._topic: GetPositionIK})

		self._move_group = move_group
		self._result = None
		self._failed = False
		self._ignore_collisions = ignore_collisions
		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		if self._failed:
			return 'failed'
			
		if self._result.error_code.val == MoveItErrorCodes.SUCCESS:
			userdata.joint_config = self._result.solution.joint_state.position
			return 'planned'
		else:
			Logger.logwarn('IK calculation resulted in error %d' % self._result.error_code.val)
			self._failed = True
			return 'failed'

			
	def on_enter(self, userdata):
		self._failed = False
		request = GetPositionIKRequest()
		request.ik_request.group_name = self._move_group
		request.ik_request.avoid_collisions = not self._ignore_collisions

		request.ik_request.pose_stamped = userdata.eef_pose

		try:
			self._result = self._srv.call(self._topic, request)
		except Exception as e:
			Logger.logwarn('Failed to calculate IK!\n%s' % str(e))
			self._failed = True

