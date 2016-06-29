#!/usr/bin/env python

import rospy

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyServiceCaller

from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import ExecuteKnownTrajectory, ExecuteKnownTrajectoryRequest

'''
Created on 10/31/2014

@author: Philipp Schillinger
'''
class MoveitExecuteTrajectoryState(EventState):
	'''
	Executes a given joint trajectory message.

	># joint_trajectory 	JointTrajectory		Trajectory to be executed.

	<= done 									Trajectory has successfully finished its execution.
	<= failed 									Failed to execute trajectory.

	'''


	def __init__(self):
		'''
		Constructor
		'''
		super(MoveitExecuteTrajectoryState, self).__init__(outcomes=['done', 'failed'],
														input_keys=['joint_trajectory'])

		self._topic = "/execute_kinematic_path"
		self._srv = ProxyServiceCaller({self._topic: ExecuteKnownTrajectory})

		self._failed = False
		self._result = None


	def execute(self, userdata):
		if self._failed:
			return 'failed'

		if self._result.error_code.val == MoveItErrorCodes.SUCCESS:
			return 'done'
		else:
			Logger.logwarn('Failed to execute trajectory: %d' % self._result.error_code.val)
			return 'failed'


	def on_enter(self, userdata):
		self._failed = False

		request = ExecuteKnownTrajectoryRequest()
		request.trajectory.joint_trajectory = userdata.joint_trajectory
		request.wait_for_execution = True
		
		try: 
			self._result = self._srv.call(self._topic, request)
		except Exception as e:
			Logger.logwarn('Was unable to execute joint trajectory:\n%s' % str(e))
			self._failed = True



