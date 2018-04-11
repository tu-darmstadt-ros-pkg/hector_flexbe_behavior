#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_manipulation_states.get_joint_values_state import GetJointValuesState
from flexbe_manipulation_states.get_joints_from_srdf_state import GetJointsFromSrdfState
from flexbe_manipulation_states.moveit_to_joints_state import MoveitToJointsState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat Jun 18 2016
@author: Philipp Schillinger
'''
class ArmTestsSM(Behavior):
	'''
	Meant to be used for testing different ways to move the arm.
	'''


	def __init__(self):
		super(ArmTestsSM, self).__init__()
		self.name = 'Arm Tests'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		srdf = "hector_tracker_robot_moveit_config/config/taurob_tracker.srdf"
		arm_joints = ['arm_joint_%d'%i for i in range(4)]
		# x:333 y:440, x:333 y:240
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:41 y:78
			OperatableStateMachine.add('Get_Current_Joints',
										GetJointValuesState(joints=arm_joints),
										transitions={'retrieved': 'Look_Up_Joints'},
										autonomy={'retrieved': Autonomy.Off},
										remapping={'joint_values': 'joints_initial'})

			# x:34 y:178
			OperatableStateMachine.add('Look_Up_Joints',
										GetJointsFromSrdfState(config_name="arm_extended_drive_pose", srdf_file=srdf, move_group="", robot_name=""),
										transitions={'retrieved': 'Extend_Arm', 'file_error': 'failed'},
										autonomy={'retrieved': Autonomy.Off, 'file_error': Autonomy.Off},
										remapping={'joint_values': 'joints_extended_drive'})

			# x:43 y:278
			OperatableStateMachine.add('Extend_Arm',
										MoveitToJointsState(move_group="arm_group", joint_names=arm_joints, action_topic='/move_group'),
										transitions={'reached': 'Retract_Arm', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.High, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'joints_extended_drive'})

			# x:43 y:428
			OperatableStateMachine.add('Retract_Arm',
										MoveitToJointsState(move_group="arm_group", joint_names=arm_joints, action_topic='/move_group'),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'joints_initial'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
