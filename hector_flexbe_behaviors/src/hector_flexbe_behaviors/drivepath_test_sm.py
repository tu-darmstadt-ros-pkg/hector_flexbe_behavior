#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hector_flexbe_states.get_robot_pose import GetRobotPose
from hector_flexbe_states.build_path import BuildPath
from hector_flexbe_states.drivepath_test import DrivepathTest
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jun 28 2016
@author: def
'''
class DrivepathTestSM(Behavior):
	'''
	abc
	'''


	def __init__(self):
		super(DrivepathTestSM, self).__init__()
		self.name = 'Drivepath Test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.poses = []

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Get Pose',
										GetRobotPose(),
										transitions={'succeeded': 'Build Path'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:256 y:42
			OperatableStateMachine.add('Build Path',
										BuildPath(),
										transitions={'succeeded': 'Test', 'next': 'Get Pose'},
										autonomy={'succeeded': Autonomy.High, 'next': Autonomy.High},
										remapping={'pose': 'pose', 'poses': 'poses'})

			# x:408 y:44
			OperatableStateMachine.add('Test',
										DrivepathTest(),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'poses': 'poses'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
