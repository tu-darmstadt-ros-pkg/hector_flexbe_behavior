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

class ServiceCallerState(EventState):
	'''
	Lets the robot move its arm.

	># joint_positions		float[]		Target joint configuration of the arm.
	># joint_names		        float[]		Joint names to move, same order as joint_positions.


	<= reached 						Target joint configuration has been reached.
	<= control_failed 					Failed to move the arm along the planned trajectory.

	'''


	def __init__(self, service_topic='/move_group/trajectory_execution/set_parameters'):
		'''
		Constructor
		'''
		super(ServiceCallerState, self).__init__(outcomes=['success', 'failed'],
											input_keys=['value'])
		
		self._service_topic = service_topic
		self._service_client = ProxyServiceCaller({self._service_topic: Reconfigure})
                
                

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
		self._service_msg = Config()
                temp_parameter = DoubleParameter()
                temp_parameter.name = 'allowed_start_tolerance'
                temp_parameter.value = userdata.value
                self._service_msg.doubles = [temp_parameter]


		try:
                        self._service_client.call(self._service_topic, self._service_msg)
                        self._success = 'True'
		except Exception as e:
			Logger.logwarn('Failed to send execute trajectory for arm motion:\n%s' % str(e))
			self._failed = True
			

	def on_stop(self):
                pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		pass
