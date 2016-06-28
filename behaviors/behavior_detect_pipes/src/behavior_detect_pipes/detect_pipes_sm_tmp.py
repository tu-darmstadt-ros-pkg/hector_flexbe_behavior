#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_detect_pipes')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_manipulation_states.get_joints_from_srdf_state import GetJointsFromSrdfState
from flexbe_manipulation_states.moveit_to_joints_state import MoveitToJointsState
from hector_flexbe_states.pipe_detection_state import PipeDetectionState
from hector_flexbe_states.monitor_percept_state import MonitorPerceptState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jun 28 2016
@author: Philipp Schillinger
'''
class DetectPipesSM(Behavior):
	'''
	Position as required and try different ways to detect the pipe positions
	'''


	def __init__(self):
		super(DetectPipesSM, self).__init__()
		self.name = 'Detect Pipes'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		joints_arm_with_gripper = ['arm_joint_%d'%i for i in range(5)]
		srdf = ""
		# x:75 y:569, x:385 y:156
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], output_keys=['pipes_pose'])
		_state_machine.userdata.pipes_pose = None

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Get_Compact_Arm_Config',
										GetJointsFromSrdfState(config_name=compact_drive_pose, srdf_file=srdf, move_group="", robot_name=""),
										transitions={'retrieved': 'Move_To_Compact_Arm', 'file_error': 'failed'},
										autonomy={'retrieved': Autonomy.Off, 'file_error': Autonomy.Off},
										remapping={'joint_values': 'joints_compact_drive'})

			# x:30 y:125
			OperatableStateMachine.add('Move_To_Compact_Arm',
										MoveitToJointsState(move_group="arm_with_gripper_group", joint_names=joints_arm_with_gripper, action_topic='/move_group'),
										transitions={'reached': 'Detect_Pipes', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'joints_compact_drive'})

			# x:45 y:318
			OperatableStateMachine.add('Detect_Pipes',
										PipeDetectionState(max_attempts=10),
										transitions={'found': 'Get_Pipes_Pose', 'unknown': 'failed'},
										autonomy={'found': Autonomy.Off, 'unknown': Autonomy.Off})

			# x:90 y:405
			OperatableStateMachine.add('Get_Pipes_Pose',
										MonitorPerceptState(required_support=0, percept_class='start_check_pipe', track_percepts=False),
										transitions={'detected': 'finished'},
										autonomy={'detected': Autonomy.Off},
										remapping={'object_id': 'object_id', 'object_pose': 'object_pose', 'object_data': 'object_data'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
