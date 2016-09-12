#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_explore_and_search_victims_using_arm')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_manipulation_states.get_joints_from_srdf_state import GetJointsFromSrdfState
from hector_flexbe_states.confirm_victim import ConfirmVictim
from hector_flexbe_states.discard_victim import DiscardVictim
from hector_flexbe_states.move_arm_dyn_state import MoveArmDynState
from flexbe_manipulation_states.moveit_to_joints_state import MoveitToJointsState
from flexbe_states.operator_decision_state import OperatorDecisionState
from behavior_approach_victim.approach_victim_sm import ApproachVictimSM
from behavior_explore.explore_sm import ExploreSM
from hector_flexbe_states.detect_object import DetectObject
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import PoseStamped
# [/MANUAL_IMPORT]


'''
Created on Thu May 19 2016
@author: Elisa und Gabriel
'''
class ExploreandSearchVictimsusingArmSM(Behavior):
	'''
	Explore and search victims
	'''


	def __init__(self):
		super(ExploreandSearchVictimsusingArmSM, self).__init__()
		self.name = 'Explore and Search Victims using Arm'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(ApproachVictimSM, 'Approach Victim')
		self.add_behavior(ExploreSM, 'Explore and Detect/Explore')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		srdf = "hector_tracker_robot_moveit_config/config/taurob_tracker.srdf"
		arm_gripper_joints = ["arm_joint_%d"%i for i in range(5)]
		# x:381 y:59
		_state_machine = OperatableStateMachine(outcomes=['failed'])
		_state_machine.userdata.pose = PoseStamped()
		_state_machine.userdata.group_name = 'complete_arm_with_gripper_group'
		_state_machine.userdata.type = 'victim'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:322, x:130 y:322, x:230 y:322
		_sm_explore_and_detect_0 = ConcurrencyContainer(outcomes=['finished'], output_keys=['victim'], conditions=[
										('finished', [('Explore', 'finished')]),
										('finished', [('Detect_Victim', 'found')])
										])

		with _sm_explore_and_detect_0:
			# x:53 y:47
			OperatableStateMachine.add('Explore',
										self.use_behavior(ExploreSM, 'Explore and Detect/Explore'),
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Inherit})

			# x:194 y:62
			OperatableStateMachine.add('Detect_Victim',
										DetectObject(),
										transitions={'found': 'finished'},
										autonomy={'found': Autonomy.Off},
										remapping={'pose': 'pose', 'victim': 'victim'})



		with _state_machine:
			# x:36 y:52
			OperatableStateMachine.add('Get_Initial_Arm_Pose',
										GetJointsFromSrdfState(config_name="compact_drive_pose", srdf_file=srdf, move_group="", robot_name=""),
										transitions={'retrieved': 'Set_Initial_Arm_Pose', 'file_error': 'failed'},
										autonomy={'retrieved': Autonomy.Off, 'file_error': Autonomy.Off},
										remapping={'joint_values': 'compact_drive_config'})

			# x:395 y:260
			OperatableStateMachine.add('Confirm_Victim',
										ConfirmVictim(),
										transitions={'confirmed': 'Set_Initial_Arm_Pose'},
										autonomy={'confirmed': Autonomy.Off},
										remapping={'victim': 'victim'})

			# x:402 y:367
			OperatableStateMachine.add('Discard_Victim',
										DiscardVictim(),
										transitions={'discarded': 'Set_Initial_Arm_Pose'},
										autonomy={'discarded': Autonomy.Off},
										remapping={'victim': 'victim'})

			# x:846 y:336
			OperatableStateMachine.add('Set_Victim_Arm_Pose',
										MoveArmDynState(),
										transitions={'reached': 'Verify_Victim', 'sampling_failed': 'Verify_Victim', 'planning_failed': 'Verify_Victim', 'control_failed': 'Verify_Victim'},
										autonomy={'reached': Autonomy.Off, 'sampling_failed': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'object_pose': 'pose', 'object_type': 'type', 'object_id': 'victim'})

			# x:36 y:178
			OperatableStateMachine.add('Set_Initial_Arm_Pose',
										MoveitToJointsState(move_group="complete_arm_with_gripper_group", joint_names=arm_gripper_joints, action_topic='/move_group'),
										transitions={'reached': 'Explore and Detect', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'compact_drive_config'})

			# x:606 y:321
			OperatableStateMachine.add('Verify_Victim',
										OperatorDecisionState(outcomes=['confirm','discard','retry'], hint=discard or confirm victim, suggestion=confirm),
										transitions={'confirm': 'Confirm_Victim', 'discard': 'Discard_Victim', 'retry': 'Approach Victim'},
										autonomy={'confirm': Autonomy.High, 'discard': Autonomy.Full, 'retry': Autonomy.Full})

			# x:843 y:166
			OperatableStateMachine.add('Approach Victim',
										self.use_behavior(ApproachVictimSM, 'Approach Victim'),
										transitions={'finished': 'Set_Victim_Arm_Pose', 'failed': 'Verify_Victim'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pose': 'pose', 'victim': 'victim'})

			# x:312 y:171
			OperatableStateMachine.add('Explore and Detect',
										_sm_explore_and_detect_0,
										transitions={'finished': 'Approach Victim'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'victim': 'victim'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
