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
		_state_machine.userdata.joint_positions2 = [-0.42]
		_state_machine.userdata.joint_names2 = ['arm_pitch_joint_1']
		_state_machine.userdata.joint_positions3 = [-1.57]
		_state_machine.userdata.joint_names3 = ['arm_roll_joint']
		_state_machine.userdata.joint_positions4 = [1.57,0]
		_state_machine.userdata.joint_names4 = ['arm_pitch_joint_1','arm_pitch_joint_0']
		_state_machine.userdata.joint_positions5 = [0]
		_state_machine.userdata.joint_names5 = ['arm_yaw_joint']
		_state_machine.userdata.low_threshold = 0
		_state_machine.userdata.joint_positions6 = [1.57,-0.51]
		_state_machine.userdata.joint_names6 = ['arm_roll_joint','gripper_servo_joint']
		_state_machine.userdata.joint_positions7 = [0]
		_state_machine.userdata.joint_names7 = ['arm_pitch_joint_1']
		_state_machine.userdata.joint_names_full = ['gripper_servo_joint', 'arm_yaw_joint', 'arm_pitch_joint_0', 'arm_pitch_joint_1', 'arm_pitch_joint_2', 'arm_roll_joint']
		_state_machine.userdata.thresholds = [5,0.2,0.3,0.2,0.2,0.2]
		_state_machine.userdata.joint_positions_startcheck = [-1.57,1.57,1.57,-1.57,0,0]
		_state_machine.userdata.joint_positions8 = [1.20]
		_state_machine.userdata.joint_names8 = ['arm_pitch_joint_0']
		_state_machine.userdata.joint_positions9 = [0.95]
		_state_machine.userdata.joint_names9 = ['arm_pitch_joint_0']

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('get_initial_position',
										flexbe_manipulation_states__GetJointValuesState(joints=['gripper_servo_joint', 'arm_yaw_joint', 'arm_pitch_joint_0', 'arm_pitch_joint_1', 'arm_pitch_joint_2', 'arm_roll_joint']),
										transitions={'retrieved': 'check_initial_position'},
										autonomy={'retrieved': Autonomy.Off},
										remapping={'joint_values': 'joint_positions_initial'})

			# x:820 y:91
			OperatableStateMachine.add('move_pitch1',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=2),
										transitions={'reached': 'move_pitch0', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions2', 'joint_names': 'joint_names2'})

			# x:838 y:596
			OperatableStateMachine.add('move_roll',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=1),
										transitions={'reached': 'move_yaw', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions3', 'joint_names': 'joint_names3'})

			# x:838 y:492
			OperatableStateMachine.add('move_pitch_1',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=3),
										transitions={'reached': 'move_roll', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions4', 'joint_names': 'joint_names4'})

			# x:630 y:637
			OperatableStateMachine.add('move_yaw',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=2),
										transitions={'reached': 'move_roll2', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions5', 'joint_names': 'joint_names5'})

			# x:193 y:645
			OperatableStateMachine.add('set_low_threshold',
										ServiceCallerState(service_topic='/move_group/trajectory_execution/set_parameters'),
										transitions={'success': 'activate_joystick', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'low_threshold'})

			# x:387 y:620
			OperatableStateMachine.add('move_roll2',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=1),
										transitions={'reached': 'move_pitch_1_2', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions6', 'joint_names': 'joint_names6'})

			# x:818 y:288
			OperatableStateMachine.add('move_pitch1_1',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=1),
										transitions={'reached': 'move_pitch0_1', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions7', 'joint_names': 'joint_names7'})

			# x:426 y:99
			OperatableStateMachine.add('set_high_threshold',
										ServiceCallerState(service_topic='/move_group/trajectory_execution/set_parameters'),
										transitions={'success': 'close_gripper', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'value': 'high_threshold'})

			# x:224 y:59
			OperatableStateMachine.add('check_initial_position',
										CheckJointsState(),
										transitions={'success': 'activate_trajectory', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'current_joint_positions': 'joint_positions_initial', 'target_joint_positions': 'joint_positions_startcheck', 'thresholds': 'thresholds'})

			# x:591 y:72
			OperatableStateMachine.add('close_gripper',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=1),
										transitions={'reached': 'move_pitch1', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions', 'joint_names': 'joint_names'})

			# x:391 y:22
			OperatableStateMachine.add('activate_trajectory',
										SwitchControllerState(service_topic='/manipulator_arm_control/controller_manager/switch_controller', trajectory=True),
										transitions={'success': 'set_high_threshold', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:25 y:651
			OperatableStateMachine.add('activate_joystick',
										SwitchControllerState(service_topic='/manipulator_arm_control/controller_manager/switch_controller', trajectory=False),
										transitions={'success': 'finished', 'failed': 'failed'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off})

			# x:819 y:182
			OperatableStateMachine.add('move_pitch0',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=1),
										transitions={'reached': 'move_pitch1_1', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions8', 'joint_names': 'joint_names8'})

			# x:839 y:399
			OperatableStateMachine.add('move_pitch0_1',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=1),
										transitions={'reached': 'move_pitch_1', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions9', 'joint_names': 'joint_names9'})

			# x:279 y:512
			OperatableStateMachine.add('move_pitch_1_2',
										DirectJointControlState(action_topic='/execute_trajectory', time_to_pose=1),
										transitions={'reached': 'set_low_threshold', 'control_failed': 'failed', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'control_failed': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_positions': 'joint_positions7', 'joint_names': 'joint_names7'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
