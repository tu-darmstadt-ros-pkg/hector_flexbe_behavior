#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient, ProxyServiceCaller

from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint, MoveItErrorCodes, ExecuteTrajectoryAction, ExecuteTrajectoryGoal

from trajectory_msgs.msg import JointTrajectoryPoint
from dynamic_reconfigure.srv import Reconfigure
from dynamic_reconfigure.msg import DoubleParameter, Config

from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest


'''
Created on 05.06.2019

@author: Gabriel Huettenberger
'''

class SwitchControllerState(EventState):
	'''
	Switch from trajectory to joystick controller and vice versa.

	># joint_positions		float[]		Target joint configuration of the arm.
	># joint_names		        float[]		Joint names to move, same order as joint_positions.


	<= reached 						Target joint configuration has been reached.
	<= control_failed 					Failed to move the arm along the planned trajectory.

	'''


	def __init__(self, service_topic='/manipulator_arm_control/controller_manager/switch_controller', trajectory=True):
		'''
		Constructor
		'''
		super(SwitchControllerState, self).__init__(outcomes=['success', 'failed'])
		
		self._service_topic = service_topic
		self._trajectory = trajectory
		self._service_client = ProxyServiceCaller({self._service_topic: SwitchController})
		
                
                

		self._success = False
		self._failed = False
		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''

		if self._success:
			return 'success'
                if self._failed:
                        return 'failed'


			
	def on_enter(self, userdata):
		self._success = False
		self._service_msg = SwitchControllerRequest()
		self._service_msg.strictness = 2
		if self._trajectory:
			self._service_msg.start_controllers = ['manipulator_arm_traj_controller', 'gripper_traj_controller']
			self._service_msg.stop_controllers = ['arm_joystick_control', 'joy_tcp_controller']
		else:
			self._service_msg.stop_controllers = ['manipulator_arm_traj_controller', 'gripper_traj_controller']
			self._service_msg.start_controllers = ['arm_joystick_control', 'joy_tcp_controller']

		try:
                        self._service_client.call(self._service_topic, self._service_msg)
                        self._success = 'True'
		except Exception as e:
			Logger.logwarn('Failed to send switch service:\n%s' % str(e))
			self._failed = True
			

	def on_stop(self):
                pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		pass
