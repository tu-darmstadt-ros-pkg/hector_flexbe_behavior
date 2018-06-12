#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient, ProxyServiceCaller

from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint, MoveItErrorCodes, ExecuteTrajectoryAction, ExecuteTrajectoryGoal

from trajectory_msgs.msg import JointTrajectoryPoint
from dynamic_reconfigure.srv import Reconfigure
from dynamic_reconfigure.msg import DoubleParameter, Config


'''
Created on 15.06.2015

@author: Gabriel Huettenberger
'''

class DirectJointControlState(EventState):
	'''
	Lets the robot move its arm.

	># joint_positions		float[]		Target joint configuration of the arm.
	># joint_names		        []		Joint names to move, same order as joint_positions.


	<= reached 						Target joint configuration has been reached.
	<= control_failed 					Failed to move the arm along the planned trajectory.

	'''


	def __init__(self, action_topic='/execute_trajectory', time_to_pose=2):
		'''
		Constructor
		'''
		super(DirectJointControlState, self).__init__(outcomes=['reached', 'control_failed', 'failed'],
											input_keys=['joint_positions', 'joint_names'])
		
		self._action_topic = '/execute_trajectory'
		self._client = ProxyActionClient({self._action_topic: ExecuteTrajectoryAction})

                self._action_goal = ExecuteTrajectoryGoal()
                self._time_to_pose = time_to_pose
              

		self._control_failed = False
		self._success = False
		self._failed = False
		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
		if self._control_failed:
			return 'control_failed'
		if self._success:
			return 'reached'
                if self._failed:
                        return 'failed'

		if self._client.has_result(self._action_topic):
			result = self._client.get_result(self._action_topic)
			
			if result.error_code.val == MoveItErrorCodes.CONTROL_FAILED:
				Logger.logwarn('Move Action failed with result error code: %s' % str(result.error_code))
				self._control_failed = True
				return 'control_failed'
			elif result.error_code.val != MoveItErrorCodes.SUCCESS:
				Logger.logwarn('Move Action failed with result error code: %s' % str(result.error_code))
				self._failed = True
				return 'failed'
			else:
				self._success = True
				return 'reached'

			
	def on_enter(self, userdata):
		self._control_failed = False
		self._success = False

                self._action_goal.trajectory.joint_trajectory.header.frame_id = 'world'
                self._action_goal.trajectory.joint_trajectory.header.stamp = rospy.get_rostime()
                self._action_goal.trajectory.joint_trajectory.joint_names = userdata.joint_names
                temp_point = JointTrajectoryPoint()
                temp_point.positions = userdata.joint_positions
                temp_point.time_from_start = rospy.Duration(self._time_to_pose)
                self._action_goal.trajectory.joint_trajectory.points = [temp_point]


		try:
			self._client.send_goal(self._action_topic, self._action_goal)
		except Exception as e:
			Logger.logwarn('Failed to send execute trajectory for arm motion:\n%s' % str(e))
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
