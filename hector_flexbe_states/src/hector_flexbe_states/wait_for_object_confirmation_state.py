#!/usr/bin/env python

import rospy
import math
import actionlib

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyActionClient

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult

'''
Created on 10/31/2014

@author: Gabriel Huettenberger
'''
class WaitForObjectConfirmationState(EventState):
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
		super(WaitForObjectConfirmationState, self).__init__(outcomes=['confirmed', 'rejected'],
														input_keys=['object_id'])

		self._action_topic = "PLACEHOLDER"
		self._client = ProxyActionClient({self._action_topic: FollowJointTrajectoryAction})

		self._confirmed = False
		self._rejected = False

		


	def execute(self, userdata):

		if self._client.has_result(self._action_topic):
			result = self._client.get_result(self._action_topic)
                            if result == True:
                                    return 'confirmed'
                            else:
                                    return 'rejected'


	def on_enter(self, userdata):
		self._confirmed = False
		self._rejected = False

		# create goal
		goal = FollowJointTrajectoryGoal()

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


