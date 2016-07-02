#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyServiceCaller, ProxyTransformListener
from moveit_msgs.srv import GetCartesianPath, GetCartesianPathRequest
from moveit_msgs.msg import MoveItErrorCodes, OrientationConstraint

'''
Created on 15.06.2015

@author: Philipp Schillinger
'''

class CalculateCartesianPathState(EventState):
	'''
	Calculates the trajectory for a cartesian path to the given goal endeffector pose.

	-- move_group 		string 				Name of the move group to be used for planning.
	-- ignore_collisions bool 				Ignore collision, use carefully.

	># eef_pose			PoseStamped			Target pose of the endeffector.

	#> joint_trajectory	JointTrajectory 	Calculated joint trajectory.
	#> plan_fraction 	float 				Fraction of the planned path.

	<= planned 								Found a plan to the target.
	<= failed 								Failed to find a plan to the given target.

	'''


	def __init__(self, move_group, ignore_collisions = False):
		'''
		Constructor
		'''
		super(CalculateCartesianPathState, self).__init__(outcomes=['planned', 'failed'],
											input_keys=['eef_pose'],
											output_keys=['joint_trajectory', 'plan_fraction'])
		
		self._topic = '/compute_cartesian_path'
		self._srv = ProxyServiceCaller({self._topic: GetCartesianPath})

		self._tf = ProxyTransformListener().listener()

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

		if self._result.error_code.val == MoveItErrorCodes.SUCCESS and self._result.fraction > .2:
			userdata.plan_fraction = self._result.fraction
			userdata.joint_trajectory = self._result.solution.joint_trajectory
			Logger.loginfo('Successfully planned %.2f of the path' % self._result.fraction)
			return 'planned'
		elif self._result.error_code.val == MoveItErrorCodes.SUCCESS:
			Logger.logwarn('Only planned %.2f of the path' % self._result.fraction)
			self._failed = True
			return 'failed'
		else:
			Logger.logwarn('Failed to plan trajectory: %d' % self._result.error_code.val)
			self._failed = True
			return 'failed'

			
	def on_enter(self, userdata):
		self._failed = False
		request = GetCartesianPathRequest()
		request.group_name = self._move_group
		request.avoid_collisions = not self._ignore_collisions
		request.max_step = 0.05

		request.header = userdata.eef_pose.header
		request.waypoints.append(userdata.eef_pose.pose)

		now = rospy.Time.now()

		try:
			self._tf.waitForTransform('base_link', 'gripper_cam_link', now, rospy.Duration(1))
			(p,q) = self._tf.lookupTransform('gripper_cam_link', 'base_link', now)

			c = OrientationConstraint()
			c.header.frame_id = 'base_link'
			c.header.stamp = now
			c.link_name = 'gripper_cam_link'
			c.orientation.x = q[0]
			c.orientation.y = q[1]
			c.orientation.z = q[2]
			c.orientation.w = q[3]
			c.weight = 1
			c.absolute_x_axis_tolerance = 0.1
			c.absolute_y_axis_tolerance = 0.1
			c.absolute_z_axis_tolerance = 0.1

			#request.path_constraints.orientation_constraints.append(c)

			self._result = self._srv.call(self._topic, request)
		except Exception as e:
			Logger.logwarn('Exception while calculating path:\n%s' % str(e))
			self._failed = True

