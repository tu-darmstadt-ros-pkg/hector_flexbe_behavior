#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient, ProxySubscriberCached

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

'''
Created on 15.06.2015

@author: Philipp Schillinger
@author: Alberto Romay
'''

class MoveFlipperState(EventState):
	'''
	Lets the robot move its flippers.

        ># flipper_1		float		Flipper position

	<= reached 					Target flipper configuration has been reached.
	<= failed 					Failed to find a plan to the given joint configuration.

	'''


	def __init__(self):
		'''
		Constructor
		'''
		super(MoveFlipperState, self).__init__(outcomes=['reached', 'failed'],
                                                                                        input_keys=['flipper_1'])
		
                self._state_topic = '/flipper_control/flipper_traj_controller/state'
		self._sub = ProxySubscriberCached({self._state_topic: JointTrajectoryControllerState})

                self._action_topic = '/flipper_control/flipper_traj_controller/follow_joint_trajectory'
		self._client = ProxyActionClient({self._action_topic: FollowJointTrajectoryAction})

		self._failed = False
		self._reached = False

		self._diff_threshold = 0.02 # rad
		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''

		if self._failed:
			return 'failed'

		if self._reached and self._sub.has_msg(self._state_topic):
			msg = self._sub.get_last_msg(self._state_topic)
			diff = max(abs(d - a) for d, a in zip(msg.desired.positions, msg.actual.positions))
			if diff < self._diff_threshold:
				return 'reached'

		if self._client.has_result(self._action_topic):
			result = self._client.get_result(self._action_topic)
			if result.error_code == result.SUCCESSFUL:
				self._reached = True
			else:
				self._failed = True
				Logger.logwarn('Failed to execute flipper command:\n(%d) %s' % (result.error_code, result.error_string))
				return 'failed'

			
	def on_enter(self, userdata):
		self._failed = False
		self._reached = False

		joint_command = JointTrajectory()
		joint_command.header.stamp = rospy.Time.now()
                joint_command.header.frame_id = 'world'
                joint_command.joint_names= ['flipper_joint_1']
                point = JointTrajectoryPoint()
                point.positions = [userdata.flipper_1]
		point.time_from_start = rospy.Duration.from_sec(2.0)
		joint_command.points.append(point)

		action_goal = FollowJointTrajectoryGoal()
		action_goal.trajectory = joint_command

		try:
			self._client.send_goal(self._action_topic, action_goal)
		except Exception as e:
			Logger.logwarn('Failed to send flipper configuration:\n%s' % str(e))
			self._failed = True
			

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
