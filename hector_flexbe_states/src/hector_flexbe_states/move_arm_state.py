#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient

from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint, MoveItErrorCodes

'''
Created on 15.06.2015

@author: Philipp Schillinger
'''

class StartTouchServoState(EventState):
	'''
	Lets the robot move its arm.

	># joint_config		float[]		Target joint configuration of the arm.

	<= reached 						Target joint configuration has been reached.
	<= planning_failed 						Failed to find a plan to the given joint configuration.
	<= control_failed 						Failed to move the arm along the planned trajectory.

	'''


	def __init__(self):
		'''
		Constructor
		'''
		super(StartTouchServoState, self).__init__(outcomes=['planning_failed', 'control_failed'],
											input_keys=['joint_config', 'group_name'])
		
		self._action_topic = '/move_group'
		self._client = ProxyActionClient({self._action_topic: MoveGroupAction})

		self._joint_names = ['arm_joint_0', 'arm_joint_1', 'arm_joint_2', 'arm_joint_3']

		self._planning_failed = False
		self._control_failed = False
		self._success = False
		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		if self._planning_failed:
			return 'planning_failed'
		if self._control_failed:
			return 'control_failed'
		if self._success:
			return 'reached'

		if self._client.has_result(self._action_topic):
			result = self._client.get_result(self._action_topic)
			
			if result.error_code.val == MoveItErrorCodes.CONTROL_FAILED:
				Logger.logwarn('Move Action failed with result error code: %s' % str(result.error_code))
				self._control_failed = True
				return 'control_failed'
			elif result.error_code.val != MoveItErrorCodes.SUCCESS:
				Logger.logwarn('Move Action failed with result error code: %s' % str(result.error_code))
				self._planning_failed = True
				return 'planning_failed'
			else:
				self._success = True
				return 'reached'

			
	def on_enter(self, userdata):
		self._planning_failed = False
		self._control_failed = False
		self._success = False

		action_goal = MoveGroupGoal()
		action_goal.request.group_name = userdata.group_name
		goal_constraints = Constraints()
		for i in range(len(self._joint_names)):
			goal_constraints.joint_constraints.append(JointConstraint(joint_name=self._joint_names[i], position=userdata.joint_config[i]))
		action_goal.request.goal_constraints.append(goal_constraints)

		try:
			self._client.send_goal(self._action_topic, action_goal)
		except Exception as e:
			Logger.logwarn('Failed to send move group goal for arm motion:\n%s' % str(e))
			self._planning_failed = True
			

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
