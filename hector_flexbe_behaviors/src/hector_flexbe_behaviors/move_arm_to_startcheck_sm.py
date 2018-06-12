#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hector_flexbe_states.service_caller_state import ServiceCallerState
from hector_flexbe_states.direct_joint_control_state import DirectJointControlState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Jun 11 2018
@author: Gabriel Huettenberger
'''
class MovearmtostartcheckSM(Behavior):
	'''
	Move arm from drive position to startcheck position
	'''


	def __init__(self):
		super(MovearmtostartcheckSM, self).__init__()
		self.name = 'Move arm to startcheck'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.joint_positions = [1.55]
		_state_machine.userdata.joint_names = ['gripper_servo_joint']
		_state_machine.userdata.high_threshold = 99
		_state_machine.userdata.joint_positions2 = [-1.16]
		_state_machine.userdata.joint_names2 = ['arm_pitch_joint_1']
		_state_machine.userdata.joint_positions3 = [-1.57]
		_state_machine.userdata.joint_names3 = ['arm_roll_joint']
		_state_machine.userdata.joint_positions4 = [1,0]
		_state_machine.userdata.joint_names4 = ['arm_pitch_joint_1','arm_pitch_joint_0']
		_state_machine.userdata.joint_positions5 = [0]
		_state_machine.userdata.joint_names5 = ['arm_yaw_joint']
		_state_machine.userdata.low_threshold = 0
		_state_machine.userdata.joint_positions6 = [-3.14]
		_state_machine.userdata.joint_names6 = ['arm_roll_joint']

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:32 y:50
			OperatableStateMachine.add('set_high_threshold',
										ServiceCallerState(service_topic='/move_group/trajectory_execution/set_parameters'),
										transitions={'success': 'close_gripper', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'high_threshold'})

			# x:248 y:52
			OperatableStateMachine.add('close_gripper',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=2),
										transitions={'reached': 'move_pitch1', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions', 'joint_names': 'joint_names'})

			# x:464 y:75
			OperatableStateMachine.add('move_pitch1',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=2),
										transitions={'reached': 'move_roll', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions2', 'joint_names': 'joint_names2'})

			# x:459 y:223
			OperatableStateMachine.add('move_roll',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=2),
										transitions={'reached': 'move_pitch_1', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions3', 'joint_names': 'joint_names3'})

			# x:477 y:387
			OperatableStateMachine.add('move_pitch_1',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=2),
										transitions={'reached': 'move_yaw', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions4', 'joint_names': 'joint_names4'})

			# x:493 y:560
			OperatableStateMachine.add('move_yaw',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=2),
										transitions={'reached': 'move_roll2', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions5', 'joint_names': 'joint_names5'})

			# x:28 y:531
			OperatableStateMachine.add('set_low_threshold',
										ServiceCallerState(service_topic='/move_group/trajectory_execution/set_parameters'),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'low_threshold'})

			# x:228 y:545
			OperatableStateMachine.add('move_roll2',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=2),
										transitions={'reached': 'set_low_threshold', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions6', 'joint_names': 'joint_names6'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
