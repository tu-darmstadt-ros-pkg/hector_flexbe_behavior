#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_manipulation_states.get_joint_values_state import GetJointValuesState
from hector_flexbe_states.direct_joint_control_state import DirectJointControlState
from hector_flexbe_states.service_caller_state import ServiceCallerState
from hector_flexbe_states.check_joints_state import CheckJointsState
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
		_state_machine.userdata.joint_positions4 = [1.57,0]
		_state_machine.userdata.joint_names4 = ['arm_pitch_joint_1','arm_pitch_joint_0']
		_state_machine.userdata.joint_positions5 = [0]
		_state_machine.userdata.joint_names5 = ['arm_yaw_joint']
		_state_machine.userdata.low_threshold = 0
		_state_machine.userdata.joint_positions6 = [-3.14,-0.51]
		_state_machine.userdata.joint_names6 = ['arm_roll_joint','gripper_servo_joint']
		_state_machine.userdata.joint_positions7 = [0]
		_state_machine.userdata.joint_names7 = ['arm_pitch_joint_1']
		_state_machine.userdata.joint_names_full = ['gripper_servo_joint', 'arm_yaw_joint', 'arm_pitch_joint_0', 'arm_pitch_joint_1', 'arm_pitch_joint_2', 'arm_roll_joint']
		_state_machine.userdata.thresholds = [3,0.05,0.05,0.05,0.02,0.02]
		_state_machine.userdata.joint_positions_startcheck = [-1.57,1.57,1.57,-1.57,0,0]

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('get_initial_position',
										GetJointValuesState(joints=['gripper_servo_joint', 'arm_yaw_joint', 'arm_pitch_joint_0', 'arm_pitch_joint_1', 'arm_pitch_joint_2', 'arm_roll_joint']),
										transitions={'retrieved': 'check_initial_position'},
										autonomy={'retrieved': Autonomy.Off},
										remapping={'joint_values': 'joint_positions_initial'})

			# x:701 y:80
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

			# x:535 y:327
			OperatableStateMachine.add('move_pitch_1',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=4),
										transitions={'reached': 'move_yaw', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions4', 'joint_names': 'joint_names4'})

			# x:539 y:447
			OperatableStateMachine.add('move_yaw',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=2),
										transitions={'reached': 'move_pitch1_1', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions5', 'joint_names': 'joint_names5'})

			# x:28 y:531
			OperatableStateMachine.add('set_low_threshold',
										ServiceCallerState(service_topic='/move_group/trajectory_execution/set_parameters'),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'low_threshold'})

			# x:197 y:558
			OperatableStateMachine.add('move_roll2',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=2),
										transitions={'reached': 'set_low_threshold', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions6', 'joint_names': 'joint_names6'})

			# x:390 y:501
			OperatableStateMachine.add('move_pitch1_1',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=2),
										transitions={'reached': 'move_roll2', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions7', 'joint_names': 'joint_names7'})

			# x:365 y:94
			OperatableStateMachine.add('set_high_threshold',
										ServiceCallerState(service_topic='/move_group/trajectory_execution/set_parameters'),
										transitions={'success': 'close_gripper', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'high_threshold'})

			# x:224 y:59
			OperatableStateMachine.add('check_initial_position',
										CheckJointsState(),
										transitions={'success': 'set_high_threshold', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'current_joint_positions': 'joint_positions_initial', 'target_joint_positions': 'joint_positions_startcheck', 'thresholds': 'thresholds'})

			# x:519 y:74
			OperatableStateMachine.add('close_gripper',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=2),
										transitions={'reached': 'move_pitch1', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions', 'joint_names': 'joint_names'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
