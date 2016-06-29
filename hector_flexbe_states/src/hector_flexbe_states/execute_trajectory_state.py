#!/usr/bin/env python

import rospy
import math
import actionlib

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from trajectory_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult

'''
Created on 10/31/2014

@author: Philipp Schillinger
'''
class ExecuteTrajectoryState(EventState):
	'''
	Executes a given joint trajectory message.

	-- controller       	string      		Namespace of the joint trajectory controller.
												The part of the topic name before /follow_joint_trajectory.

	># joint_trajectory 	JointTrajectory		Trajectory to be executed, containing all required information.

	<= done 									Trajectory has successfully finished its execution.
	<= failed 									Failed to execute trajectory.

	'''


	def __init__(self, controller):
		'''
		Constructor
		'''
		super(ExecuteTrajectoryState, self).__init__(outcomes=['done', 'failed'],
														input_keys=['joint_trajectory'])

		self._action_topic = controller + "/follow_joint_trajectory"
		self._client = ProxyActionClient({self._action_topic: FollowJointTrajectoryAction})

		self._failed = False


	def execute(self, userdata):
		if self._failed:
			return 'failed'

		if self._client.has_result(self._action_topic):
			result = self._client.get_result(self._action_topic)
			if result:
				if result.error_code == FollowJointTrajectoryResult.SUCCESSFUL:
					return 'done'
				else:
					Logger.logwarn('Joint trajectory request failed to execute (%d). Reason: %s' % (result.error_code, result.error_string))
					self._failed = True
					return 'failed'
			else:
				Logger.logwarn('Wait for result returned True even though the result is %s' % str(result))
				self._failed = True
				return 'failed'


	def on_enter(self, userdata):
		self._failed = False

		# create goal
		goal = FollowJointTrajectoryGoal()
		goal.trajectory = userdata.joint_trajectory
		goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)

		# execute the motion
		try: 
			self._client.send_goal(self._action_topic, goal)
		except Exception as e:
			Logger.logwarn('Was unable to execute joint trajectory:\n%s' % str(e))
			self._failed = True


	def on_exit(self, userdata):
		if not self._client.has_result(self._action_topic):
			self._client.cancel(self._action_topic)
			Logger.loginfo("Cancelled active action goal.")


