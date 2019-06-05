#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_manipulation_states.get_joint_values_state import GetJointValuesState as flexbe_manipulation_states__GetJointValuesState
from hector_flexbe_states.direct_joint_control_state import DirectJointControlState
from hector_flexbe_states.service_caller_state import ServiceCallerState
from hector_flexbe_states.check_joints_state import CheckJointsState
from hector_flexbe_states.switch_controller_state import SwitchControllerState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jun 12 2018
@author: Gabriel Huettenberger
'''
class MovearmtoparkSM(Behavior):
	'''
	Moves the arm to the park position
	'''


	def __init__(self):
		super(MovearmtoparkSM, self).__init__()
		self.name = 'Move arm to park'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.joint_names = ['arm_pitch_joint_0']
		_state_machine.userdata.joint_positions = [0]
		_state_machine.userdata.joint_names2 = ['arm_pitch_joint_1','arm_pitch_joint_2','arm_roll_joint','gripper_servo_joint']
		_state_machine.userdata.joint_positions2 = [1.57,0,-1.57,1.55]
		_state_machine.userdata.joint_names3 = ['arm_yaw_joint']
		_state_machine.userdata.joint_positions3 = [1.57]
		_state_machine.userdata.joint_names4 = ['arm_pitch_joint_1','arm_pitch_joint_0']
		_state_machine.userdata.joint_positions4 = [-1.16,1.57]
		_state_machine.userdata.joint_names5 = ['arm_roll_joint']
		_state_machine.userdata.joint_positions5 = [0]
		_state_machine.userdata.joint_names6 = ['gripper_servo_joint']
		_state_machine.userdata.joint_positions6 = [-1.57]
		_state_machine.userdata.high_value = 99
		_state_machine.userdata.low_value = 0
		_state_machine.userdata.joint_names7 = ['arm_pitch_joint_1']
		_state_machine.userdata.joint_positions7 = [-1.57]
		_state_machine.userdata.joint_positions_park = [-0.5,0,0,0,0,-3.13]
		_state_machine.userdata.thresholds = [5,0.75,1.2,1,1.5,10]

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:44
			OperatableStateMachine.add('get_initial_position',
										flexbe_manipulation_states__GetJointValuesState(joints=['gripper_servo_joint', 'arm_yaw_joint', 'arm_pitch_joint_0', 'arm_pitch_joint_1', 'arm_pitch_joint_2', 'arm_roll_joint']),
										transitions={'retrieved': 'activate_trajectory'},
										autonomy={'retrieved': Autonomy.Off},
										remapping={'joint_values': 'joint_positions_initial'})

			# x:602 y:135
			OperatableStateMachine.add('move_multiple',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=2),
										transitions={'reached': 'move_yaw', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions2', 'joint_names': 'joint_names2'})

			# x:604 y:258
			OperatableStateMachine.add('move_yaw',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=2),
										transitions={'reached': 'move_pitch_1_2', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions3', 'joint_names': 'joint_names3'})

			# x:601 y:378
			OperatableStateMachine.add('move_pitch_1_2',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=1),
										transitions={'reached': 'move_roll', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions4', 'joint_names': 'joint_names4'})

			# x:529 y:504
			OperatableStateMachine.add('move_roll',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=1),
										transitions={'reached': 'move_pitch_1', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions5', 'joint_names': 'joint_names5'})

			# x:236 y:507
			OperatableStateMachine.add('move_gripper',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=1),
										transitions={'reached': 'set_low_threshold', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions6', 'joint_names': 'joint_names6'})

			# x:0 y:602
			OperatableStateMachine.add('set_low_threshold',
										ServiceCallerState(service_topic='/move_group/trajectory_execution/set_parameters'),
										transitions={'success': 'activate_joystick', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'low_value'})

			# x:487 y:43
			OperatableStateMachine.add('move_pitch0',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=1),
										transitions={'reached': 'move_multiple', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions', 'joint_names': 'joint_names'})

			# x:356 y:581
			OperatableStateMachine.add('move_pitch_1',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=1),
										transitions={'reached': 'move_gripper', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions7', 'joint_names': 'joint_names7'})

			# x:33 y:140
			OperatableStateMachine.add('check_initial_position',
										CheckJointsState(),
										transitions={'success': 'set_high_threshold', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'current_joint_positions': 'joint_positions_initial', 'target_joint_positions': 'joint_positions_park', 'thresholds': 'thresholds'})

			# x:291 y:56
			OperatableStateMachine.add('set_high_threshold',
										ServiceCallerState(service_topic='/move_group/trajectory_execution/set_parameters'),
										transitions={'success': 'move_pitch0', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'high_value'})

			# x:203 y:132
			OperatableStateMachine.add('activate_trajectory',
										SwitchControllerState(service_topic='/manipulator_arm_control/controller_manager/switch_controller', trajectory=True),
										transitions={'success': 'set_high_threshold', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:34 y:511
			OperatableStateMachine.add('activate_joystick',
										SwitchControllerState(service_topic='/manipulator_arm_control/controller_manager/switch_controller', trajectory=False),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
