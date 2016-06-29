#!/usr/bin/env python

import rospy
import math
import actionlib

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectoryPoint

'''
Created on 10/31/2014

@author: Philipp Schillinger
'''
class GripperState(EventState):
	'''
	Open or close the gripper.

	-- close_fraction 	float 	OPEN, CLOSE or any value in [0,1].
	-- duration 		float 	Time (sec) for executing the motion.

	<= done 					Trajectory has successfully finished its execution.
	<= failed 					Failed to execute trajectory.

	'''
	OPEN = 0
	CLOSE = 1

	def __init__(self, action, duration = 1.0):
		'''
		Constructor
		'''
		super(GripperState, self).__init__(outcomes=['done', 'failed'])

		self._action_topic = "/gripper_control/gripper_grasp_traj_controller/follow_joint_trajectory"
		self._client = ProxyActionClient({self._action_topic: FollowJointTrajectoryAction})

		self._failed = False
		self._duration = duration
		self._target_joint_value = action * 1.89


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

		# create point
		endpoint = JointTrajectoryPoint()
		endpoint.positions.append(self._target_joint_value)
		endpoint.time_from_start = rospy.Duration(0.2 + self._duration)

		# create goal
		goal = FollowJointTrajectoryGoal()
		goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
		goal.trajectory.joint_names.append('gripper_joint_0')
		goal.trajectory.points.append(endpoint)

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


