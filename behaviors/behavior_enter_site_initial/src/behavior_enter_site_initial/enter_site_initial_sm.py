#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_enter_site_initial')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hector_flexbe_states.move_flipper_state import MoveFlipperState
from hector_flexbe_states.drive_predefined_state import DrivePredefinedState
from hector_flexbe_states.monitor_imu_state import MonitorIMUState
from hector_flexbe_states.move_arm_state import MoveArmState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
# [/MANUAL_IMPORT]


'''
Created on Wed Jun 17 2015
@author: Philipp Schillinger
'''
class EnterSiteInitialSM(Behavior):
	'''
	Initially enter the site by crossing the step without localization. Assumes this to be step_1 and localizes afterwards.
	'''


	def __init__(self):
		super(EnterSiteInitialSM, self).__init__()
		self.name = 'Enter Site Initial'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		before_step_timeout = 25 # sec
		on_step_timeout = 13 # sec
		after_step_timeout = 1 # sec
		# x:290 y:398, x:549 y:193
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.above_step_tilt_threshold = 0.2
		_state_machine.userdata.step_speed = 0.2 # always forward from docking
		_state_machine.userdata.step_tilt_threshold = 0.2
		_state_machine.userdata.flipper_step = -0.5
		_state_machine.userdata.zero_speed = 0
		_state_machine.userdata.flipper_ready = 0.75
		_state_machine.userdata.initialpose = Pose() # manual section
		_state_machine.userdata.flipper_drive = 0.0
		_state_machine.userdata.COM_joint_config = [0, 2.3, 1.0, 0.7]
		_state_machine.userdata.drive_joint_config = [0, 0.7, 0.7,0]
		_state_machine.userdata.group_name = 'arm_group'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		_state_machine.userdata.initialpose.position.x = 7.0
		_state_machine.userdata.initialpose.position.y = 4.4
		_state_machine.userdata.initialpose.position.z = 0.0
		_state_machine.userdata.initialpose.orientation.x = 0
		_state_machine.userdata.initialpose.orientation.y = 0
		_state_machine.userdata.initialpose.orientation.z = 0
		_state_machine.userdata.initialpose.orientation.w = 1
		# [/MANUAL_CREATE]

		# x:30 y:388, x:130 y:388, x:230 y:388, x:330 y:388, x:430 y:388, x:530 y:388
		_sm_flipper_arm_drive_config_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['drive_joint_config', 'flipper_drive', 'group_name'], conditions=[
										('finished', [('Move_COM_Back', 'reached'), ('Flipper_Drive', 'reached')]),
										('failed', [('Move_COM_Back', 'planning_failed')]),
										('failed', [('Move_COM_Back', 'control_failed')]),
										('failed', [('Flipper_Drive', 'failed')])
										])

		with _sm_flipper_arm_drive_config_0:
			# x:109 y:133
			OperatableStateMachine.add('Move_COM_Back',
										MoveArmState(),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Low, 'planning_failed': Autonomy.High, 'control_failed': Autonomy.High},
										remapping={'joint_config': 'drive_joint_config', 'group_name': 'group_name'})

			# x:512 y:104
			OperatableStateMachine.add('Flipper_Drive',
										MoveFlipperState(),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'flipper_1': 'flipper_drive'})


		# x:30 y:388, x:130 y:388, x:230 y:388, x:330 y:388, x:430 y:388, x:530 y:388
		_sm_flipper_arm_step_config_1 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['COM_joint_config', 'flipper_step', 'group_name'], conditions=[
										('finished', [('Move_COM_Forward', 'reached'), ('Flipper_Step_Config', 'reached')]),
										('failed', [('Move_COM_Forward', 'planning_failed')]),
										('failed', [('Move_COM_Forward', 'control_failed')]),
										('failed', [('Flipper_Step_Config', 'failed')])
										])

		with _sm_flipper_arm_step_config_1:
			# x:80 y:139
			OperatableStateMachine.add('Move_COM_Forward',
										MoveArmState(),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Low, 'planning_failed': Autonomy.High, 'control_failed': Autonomy.High},
										remapping={'joint_config': 'COM_joint_config', 'group_name': 'group_name'})

			# x:412 y:148
			OperatableStateMachine.add('Flipper_Step_Config',
										MoveFlipperState(),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'flipper_1': 'flipper_step'})


		# x:30 y:388, x:130 y:388, x:230 y:388, x:330 y:388
		_sm_approach_step_2 = ConcurrencyContainer(outcomes=['timeout', 'tilted'], input_keys=['step_tilt_threshold', 'step_speed'], conditions=[
										('timeout', [('Drive_Forward', 'done')]),
										('tilted', [('Monitor_Tilted', 'triggered')])
										])

		with _sm_approach_step_2:
			# x:64 y:94
			OperatableStateMachine.add('Monitor_Tilted',
										MonitorIMUState(operation=MonitorIMUState.GREATER),
										transitions={'triggered': 'tilted'},
										autonomy={'triggered': Autonomy.Low},
										remapping={'tilt_threshold': 'step_tilt_threshold'})

			# x:334 y:110
			OperatableStateMachine.add('Drive_Forward',
										DrivePredefinedState(duration=before_step_timeout),
										transitions={'done': 'timeout'},
										autonomy={'done': Autonomy.Off},
										remapping={'speed': 'step_speed'})


		# x:30 y:388, x:130 y:388, x:230 y:388, x:330 y:388
		_sm_drive_over_3 = ConcurrencyContainer(outcomes=['finished', 'timeout'], input_keys=['step_speed', 'above_step_tilt_threshold'], conditions=[
										('timeout', [('Drive', 'done')]),
										('finished', [('Monitor_Tilted', 'triggered')])
										])

		with _sm_drive_over_3:
			# x:512 y:83
			OperatableStateMachine.add('Monitor_Tilted',
										MonitorIMUState(operation=MonitorIMUState.LESS),
										transitions={'triggered': 'finished'},
										autonomy={'triggered': Autonomy.Low},
										remapping={'tilt_threshold': 'above_step_tilt_threshold'})

			# x:51 y:104
			OperatableStateMachine.add('Drive',
										DrivePredefinedState(duration=on_step_timeout),
										transitions={'done': 'timeout'},
										autonomy={'done': Autonomy.Off},
										remapping={'speed': 'step_speed'})


		# x:952 y:364, x:718 y:241
		_sm_cross_step_4 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['step_tilt_threshold', 'step_speed', 'zero_speed', 'above_step_tilt_threshold', 'flipper_drive', 'flipper_ready', 'flipper_step', 'COM_joint_config', 'drive_joint_config', 'group_name'])

		with _sm_cross_step_4:
			# x:70 y:49
			OperatableStateMachine.add('Flipper_Ready_Config',
										MoveFlipperState(),
										transitions={'reached': 'Approach_Step', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'flipper_1': 'flipper_ready'})

			# x:267 y:457
			OperatableStateMachine.add('Stop_Robot_After',
										DrivePredefinedState(duration=1),
										transitions={'done': 'Flipper_Arm_Drive_Config'},
										autonomy={'done': Autonomy.Off},
										remapping={'speed': 'zero_speed'})

			# x:84 y:450
			OperatableStateMachine.add('Drive_Over',
										_sm_drive_over_3,
										transitions={'finished': 'Stop_Robot_After', 'timeout': 'Stop_Robot_After'},
										autonomy={'finished': Autonomy.Inherit, 'timeout': Autonomy.Inherit},
										remapping={'step_speed': 'step_speed', 'above_step_tilt_threshold': 'above_step_tilt_threshold'})

			# x:58 y:144
			OperatableStateMachine.add('Approach_Step',
										_sm_approach_step_2,
										transitions={'timeout': 'E_Stop_Robot', 'tilted': 'Stop_Robot_Before'},
										autonomy={'timeout': Autonomy.Inherit, 'tilted': Autonomy.Inherit},
										remapping={'step_tilt_threshold': 'step_tilt_threshold', 'step_speed': 'step_speed'})

			# x:281 y:180
			OperatableStateMachine.add('E_Stop_Robot',
										DrivePredefinedState(duration=1),
										transitions={'done': 'Stop_Robot_Before'},
										autonomy={'done': Autonomy.High},
										remapping={'speed': 'zero_speed'})

			# x:73 y:239
			OperatableStateMachine.add('Stop_Robot_Before',
										DrivePredefinedState(duration=1),
										transitions={'done': 'Flipper_Arm_Step_Config'},
										autonomy={'done': Autonomy.Off},
										remapping={'speed': 'zero_speed'})

			# x:100 y:325
			OperatableStateMachine.add('Flipper_Arm_Step_Config',
										_sm_flipper_arm_step_config_1,
										transitions={'finished': 'Drive_Over', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'COM_joint_config': 'COM_joint_config', 'flipper_step': 'flipper_step', 'group_name': 'group_name'})

			# x:534 y:395
			OperatableStateMachine.add('Flipper_Arm_Drive_Config',
										_sm_flipper_arm_drive_config_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'drive_joint_config': 'drive_joint_config', 'flipper_drive': 'flipper_drive', 'group_name': 'group_name'})



		with _state_machine:
			# x:93 y:173
			OperatableStateMachine.add('Cross_Step',
										_sm_cross_step_4,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'step_tilt_threshold': 'step_tilt_threshold', 'step_speed': 'step_speed', 'zero_speed': 'zero_speed', 'above_step_tilt_threshold': 'above_step_tilt_threshold', 'flipper_drive': 'flipper_drive', 'flipper_ready': 'flipper_ready', 'flipper_step': 'flipper_step', 'COM_joint_config': 'COM_joint_config', 'drive_joint_config': 'drive_joint_config', 'group_name': 'group_name'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
