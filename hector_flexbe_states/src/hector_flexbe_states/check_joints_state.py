#!/usr/bin/env python

import rospy
from flexbe_core import EventState, Logger

from flexbe_core.proxy import ProxyActionClient, ProxyServiceCaller

from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal, Constraints, JointConstraint, MoveItErrorCodes, ExecuteTrajectoryAction, ExecuteTrajectoryGoal

from trajectory_msgs.msg import JointTrajectoryPoint
from dynamic_reconfigure.srv import Reconfigure
from dynamic_reconfigure.msg import DoubleParameter, Config


'''
Created on 15.06.2018

@author: Gabriel Huettenberger
'''

class CheckJointsState(EventState):
	'''
	Lets the robot move its arm.

	># current_joint_positions		float[]		The current joint positions
	># target_joint_positions		float[]		Joint positions to check against, same order as current_joint_positions
	># thresholds                           float[]         Maximum difference between current_joint_positions and target_joint_positions for each joint to still return success, same order as current_joint_positions


	<= success 						current_joint_positions are the same as target_joint_positions with respect to thresholds.
	<= failed 					        current_joint_positions are farther away from target_joint_positions than thresholds

	'''


	def __init__(self):
		'''
		Constructor
		'''
		super(CheckJointsState, self).__init__(outcomes=['success', 'failed'], input_keys=['current_joint_positions', 'target_joint_positions', 'thresholds'])
		
              
		self._success = False
		self._failed = False
		
		
	def execute(self, userdata):
		'''
		Execute this state
		'''
                if self._failed:
                        return 'failed'
		if self._success:
			return 'success'




			
	def on_enter(self, userdata):
                x = 0
                while x < len(userdata.current_joint_positions):
                    if(abs(userdata.current_joint_positions[x] - userdata.target_joint_positions[x]) > userdata.thresholds[x]):
                        self._failed = True
                    Logger.loginfo(abs(userdata.current_joint_positions[x] - userdata.target_joint_positions[x]))
                    x = x+1
                self._success = True
                    

	def on_stop(self):
                pass

	def on_pause(self):
		pass

	def on_resume(self, userdata):
		pass
