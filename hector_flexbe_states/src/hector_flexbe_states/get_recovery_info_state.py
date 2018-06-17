#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient, ProxyServiceCaller

from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint, MoveItErrorCodes, ExecuteTrajectoryAction, ExecuteTrajectoryGoal

from trajectory_msgs.msg import JointTrajectoryPoint
from hector_nav_msgs.srv import GetRecoveryInfo, GetRecoveryInfoRequest, GetRecoveryInfoResponse


'''
Created on 15.06.2015

@author: Gabriel Huettenberger
'''

class GetRecoveryInfoState(EventState):
	'''
	Lets the robot move its arm.

	># joint_positions		float[]		Target joint configuration of the arm.
	># joint_names		        float[]		Joint names to move, same order as joint_positions.


	<= reached 						Target joint configuration has been reached.
	<= control_failed 					Failed to move the arm along the planned trajectory.

	'''


	def __init__(self, service_topic='/trajectory_recovery_info'):
		'''
		Constructor
		'''
		super(GetRecoveryInfoState, self).__init__(outcomes=['success', 'failed'], output_keys=['waypoint'])
		
		self._service_topic = service_topic
		self._service_client = ProxyServiceCaller({self._service_topic: GetRecoveryInfo})
                self._service_response = GetRecoveryInfoResponse()
                

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
		self._service_msg = GetRecoveryInfoRequest()
		self._service_msg.request_time = rospy.Time.now()
		self._service_msg.request_radius = 0.3


		try:
                        self._service_response = self._service_client.call(self._service_topic, self._service_msg)
                        userdata.waypoint = self._service_response.radius_entry_pose
                        self._success = 'True'
                        
		except Exception as e:
			Logger.logwarn('Failed to get recovery info:\n%s' % str(e))
			self._failed = True
			

	def on_stop(self):
                pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		pass
