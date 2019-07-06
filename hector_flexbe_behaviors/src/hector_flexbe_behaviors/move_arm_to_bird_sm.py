#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hector_flexbe_states.switch_controller_state import SwitchControllerState
from hector_flexbe_states.service_caller_state import ServiceCallerState
from hector_flexbe_states.direct_joint_control_state import DirectJointControlState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jun 19 2018
@author: Gabriel Huettenberger
'''
class MovearmtobirdSM(Behavior):
	'''
	Moves the arm to the initial startcheck position when already unfolded
	'''


	def __init__(self):
		super(MovearmtobirdSM, self).__init__()
		self.name = 'Move arm to bird'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.joint_positions = [-0.19,0,0.3,-1.91,-1.57]
		_state_machine.userdata.joint_names = ['arm_yaw_joint', 'arm_pitch_joint_0', 'arm_pitch_joint_1', 'arm_pitch_joint_2', 'arm_roll_joint']
		_state_machine.userdata.high_threshold = 99
		_state_machine.userdata.low_threshold = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:147
			OperatableStateMachine.add('activate_trajectory',
										SwitchControllerState(service_topic='/manipulator_arm_control/controller_manager/switch_controller', trajectory=True),
										transitions={'success': 'set_high_threshold', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:341 y:223
			OperatableStateMachine.add('set_low_threshold',
										ServiceCallerState(service_topic='/move_group/trajectory_execution/set_parameters'),
										transitions={'success': 'activate_joystick', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'low_threshold'})

			# x:265 y:49
			OperatableStateMachine.add('move_multiple',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=2),
										transitions={'reached': 'set_low_threshold', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions', 'joint_names': 'joint_names'})

			# x:152 y:441
			OperatableStateMachine.add('activate_joystick',
										SwitchControllerState(service_topic='/manipulator_arm_control/controller_manager/switch_controller', trajectory=False),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:74 y:85
			OperatableStateMachine.add('set_high_threshold',
										ServiceCallerState(service_topic='/move_group/trajectory_execution/set_parameters'),
										transitions={'success': 'move_multiple', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'high_threshold'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
