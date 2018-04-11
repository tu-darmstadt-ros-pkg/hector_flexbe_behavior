#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_manipulation_states.get_joints_from_srdf_state import GetJointsFromSrdfState
from hector_flexbe_states.confirm_victim import ConfirmVictim
from hector_flexbe_states.discard_victim import DiscardVictim
from hector_flexbe_states.Decide_If_Victim import DecideIfVictim
from hector_flexbe_behaviors.exploration_sm import ExplorationSM
from hector_flexbe_states.detect_object import DetectObject
from hector_flexbe_states.move_arm_dyn_state import MoveArmDynState
from flexbe_manipulation_states.moveit_to_joints_state import MoveitToJointsState
from hector_flexbe_behaviors.approach_victim_sm import ApproachVictimSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import PoseStamped
# [/MANUAL_IMPORT]


'''
Created on Thu May 19 2016
@author: Elisa und Gabriel
'''
class SearchVictimsSM(Behavior):
	'''
	Explore and search victims
	'''


	def __init__(self):
		super(SearchVictimsSM, self).__init__()
		self.name = 'Search Victims'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(ExplorationSM, 'Explore and Detect/Explore')
		self.add_behavior(ApproachVictimSM, 'Approach Victim')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		srdf = "hector_tracker_robot_moveit_config/config/taurob_tracker.srdf"
		arm_gripper_joints = ["arm_joint_%d"%i for i in range(5)]
		# x:349 y:36
		_state_machine = OperatableStateMachine(outcomes=['failed'])
		_state_machine.userdata.pose = PoseStamped()
		_state_machine.userdata.group_name = 'complete_arm_with_gripper_group'
		_state_machine.userdata.type = 'hotspot'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365
		_sm_explore_and_detect_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], output_keys=['pose', 'victim'], conditions=[
										('finished', [('Explore', 'finished')]),
										('failed', [('Explore', 'failed')]),
										('finished', [('Detect_Victim', 'found')])
										])

		with _sm_explore_and_detect_0:
			# x:40 y:44
			OperatableStateMachine.add('Explore',
										self.use_behavior(ExplorationSM, 'Explore and Detect/Explore'),
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Inherit})

			# x:346 y:59
			OperatableStateMachine.add('Detect_Victim',
										DetectObject(),
										transitions={'found': 'finished'},
										autonomy={'found': Autonomy.Off},
										remapping={'pose': 'pose', 'victim': 'victim'})



		with _state_machine:
			# x:44 y:135
			OperatableStateMachine.add('Get_Initial_Arm_Pose',
										GetJointsFromSrdfState(config_name="compact_drive_pose", srdf_file=srdf, move_group="", robot_name=""),
										transitions={'retrieved': 'Set_Initial_Arm_Pose', 'file_error': 'failed'},
										autonomy={'retrieved': Autonomy.Off, 'file_error': Autonomy.Off},
										remapping={'joint_values': 'compact_drive_config'})

			# x:515 y:251
			OperatableStateMachine.add('Confirm_Victim',
										ConfirmVictim(),
										transitions={'confirmed': 'Set_Initial_Arm_Pose'},
										autonomy={'confirmed': Autonomy.Off},
										remapping={'victim': 'victim'})

			# x:513 y:330
			OperatableStateMachine.add('Discard_Victim',
										DiscardVictim(),
										transitions={'discarded': 'Set_Initial_Arm_Pose'},
										autonomy={'discarded': Autonomy.Off},
										remapping={'victim': 'victim'})

			# x:709 y:245
			OperatableStateMachine.add('Decide_If_Victim',
										DecideIfVictim(),
										transitions={'confirm': 'Confirm_Victim', 'discard': 'Discard_Victim', 'retry': 'Set_Initial_Arm_Pose_2'},
										autonomy={'confirm': Autonomy.Full, 'discard': Autonomy.Full, 'retry': Autonomy.Full})

			# x:789 y:149
			OperatableStateMachine.add('Explore and Detect',
										_sm_explore_and_detect_0,
										transitions={'finished': 'Approach Victim', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pose': 'pose', 'victim': 'victim'})

			# x:1067 y:340
			OperatableStateMachine.add('Set_Victim_Arm_Pose',
										MoveArmDynState(),
										transitions={'reached': 'Decide_If_Victim', 'sampling_failed': 'Decide_If_Victim', 'planning_failed': 'Decide_If_Victim', 'control_failed': 'Decide_If_Victim'},
										autonomy={'reached': Autonomy.Off, 'sampling_failed': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'object_pose': 'pose', 'object_type': 'type', 'object_id': 'victim'})

			# x:304 y:142
			OperatableStateMachine.add('Set_Initial_Arm_Pose',
										MoveitToJointsState(move_group="complete_arm_with_gripper_group", joint_names=arm_gripper_joints, action_topic='/move_group'),
										transitions={'reached': 'Explore and Detect', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'compact_drive_config'})

			# x:684 y:41
			OperatableStateMachine.add('Set_Initial_Arm_Pose_2',
										MoveitToJointsState(move_group="complete_arm_with_gripper_group", joint_names=arm_gripper_joints, action_topic='/move_group'),
										transitions={'reached': 'Approach Victim', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'compact_drive_config'})

			# x:1071 y:157
			OperatableStateMachine.add('Approach Victim',
										self.use_behavior(ApproachVictimSM, 'Approach Victim'),
										transitions={'finished': 'Set_Victim_Arm_Pose', 'failed': 'Decide_If_Victim'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pose': 'pose', 'victim': 'victim'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
