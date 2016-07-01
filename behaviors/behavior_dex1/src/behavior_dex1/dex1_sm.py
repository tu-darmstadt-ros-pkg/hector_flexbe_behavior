#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_dex1')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_manipulation_states.get_joints_from_srdf_state import GetJointsFromSrdfState
from flexbe_manipulation_states.moveit_to_joints_state import MoveitToJointsState
from hector_flexbe_states.monitor_percept_state import MonitorPerceptState
from flexbe_utility_states.publish_pose_state import PublishPoseState
from hector_flexbe_states.pipe_detection_state import PipeDetectionState
from flexbe_states.calculation_state import CalculationState
from hector_flexbe_states.calculate_ik_state import CalculateIKState
from hector_flexbe_states.gripper_roll_state import GripperRollState
from hector_flexbe_states.moveit_execute_trajectory_state import MoveitExecuteTrajectoryState
from hector_flexbe_states.calculate_cartesian_path_state import CalculateCartesianPathState
from flexbe_states.operator_decision_state import OperatorDecisionState
from hector_flexbe_states.gripper_state import GripperState
from flexbe_states.log_state import LogState
from hector_flexbe_states.LookAtWaypoint import LookAtWaypoint
from hector_flexbe_states.select_pipe_pose import SelectPipePose
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from flexbe_core.proxy import ProxyTransformListener
# [/MANUAL_IMPORT]


'''
Created on Tue Jun 28 2016
@author: Philipp Schillinger
'''
class Dex1SM(Behavior):
	'''
	Position at pipes and offer choices for manipulation and inspection
	'''


	def __init__(self):
		super(Dex1SM, self).__init__()
		self.name = 'Dex1'

		# parameters of this behavior
		self.add_parameter('pose_offset', 0.25)
		self.add_parameter('close_offset', 0.05)
		self.add_parameter('pipes_side', 'right')

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		self._tf = ProxyTransformListener().listener()
		# [/MANUAL_INIT]

		# Behavior comments:

		# O 191 423 
		# Run in full autonomy to skip looking, run in high to decide



	def create(self):
		joints_arm_with_gripper = ['arm_joint_%d'%i for i in range(5)]
		srdf = "hector_tracker_robot_moveit_config/config/taurob_tracker.srdf"
		percept_class = 'pipes'
		move_group = "arm_with_gripper_group"
		move_group_no_eef = "arm_group"
		obs_pose_side = 'look_down_%s_pose' % self.pipes_side
		# x:51 y:405, x:503 y:76
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.object_pose = None
		_state_machine.userdata.object_type = percept_class
		_state_machine.userdata.test_pregrasp = [0, 1.57, 0.77, 0.38, 0]
		_state_machine.userdata.index = -1

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:569 y:352, x:45 y:183
		_sm_gripper_action_0 = OperatableStateMachine(outcomes=['finished', 'failed', 'back'])

		with _sm_gripper_action_0:
			# x:168 y:53
			OperatableStateMachine.add('Decide_Close_Gripper',
										OperatorDecisionState(outcomes=['close_gripper', 'move_back'], hint="Grasp pipe?", suggestion='close_gripper'),
										transitions={'close_gripper': 'Close_Gripper', 'move_back': 'back'},
										autonomy={'close_gripper': Autonomy.Full, 'move_back': Autonomy.Full})

			# x:221 y:173
			OperatableStateMachine.add('Close_Gripper',
										GripperState(action=0.8, duration=2.0),
										transitions={'done': 'Manipulate_Pipe', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.High})

			# x:229 y:391
			OperatableStateMachine.add('Decide_Open_Gripper',
										OperatorDecisionState(outcomes=['open_gripper', 'move_back'], hint="Open gripper or pull pipe?", suggestion='open_gripper'),
										transitions={'open_gripper': 'Open_Gripper', 'move_back': 'back'},
										autonomy={'open_gripper': Autonomy.High, 'move_back': Autonomy.Full})

			# x:241 y:290
			OperatableStateMachine.add('Manipulate_Pipe',
										LogState(text="Perform desired pipe manipulation", severity=Logger.REPORT_HINT),
										transitions={'done': 'Decide_Open_Gripper'},
										autonomy={'done': Autonomy.High})

			# x:215 y:511
			OperatableStateMachine.add('Open_Gripper',
										GripperState(action=GripperState.OPEN, duration=1.0),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.High})


		# x:153 y:384, x:388 y:221
		_sm_move_back_out_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['pipes_out_pose'])

		with _sm_move_back_out_1:
			# x:118 y:87
			OperatableStateMachine.add('Get_Cartesian_Path_Back',
										CalculateCartesianPathState(move_group=move_group, ignore_collisions=True),
										transitions={'planned': 'Execute_Cartesian_Path_Back', 'failed': 'failed'},
										autonomy={'planned': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'eef_pose': 'pipes_out_pose', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:116 y:216
			OperatableStateMachine.add('Execute_Cartesian_Path_Back',
										MoveitExecuteTrajectoryState(),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})


		# x:230 y:517, x:550 y:229
		_sm_move_to_close_pose_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['pipes_pose'])

		with _sm_move_to_close_pose_2:
			# x:97 y:43
			OperatableStateMachine.add('Move_Pose_Cartesian',
										CalculationState(calculation=lambda p: self.update_pose(p, self.close_offset)),
										transitions={'done': 'Visualize_Pipes_Close_Pose'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pipes_pose', 'output_value': 'pipes_close_pose'})

			# x:113 y:140
			OperatableStateMachine.add('Visualize_Pipes_Close_Pose',
										PublishPoseState(topic='/pipes/close_pose'),
										transitions={'done': 'Get_Cartesian_Path'},
										autonomy={'done': Autonomy.Low},
										remapping={'pose': 'pipes_close_pose'})

			# x:147 y:371
			OperatableStateMachine.add('Execute_Cartesian_Path',
										MoveitExecuteTrajectoryState(),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:121 y:260
			OperatableStateMachine.add('Get_Cartesian_Path',
										CalculateCartesianPathState(move_group=move_group, ignore_collisions=True),
										transitions={'planned': 'Execute_Cartesian_Path', 'failed': 'failed'},
										autonomy={'planned': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'eef_pose': 'pipes_close_pose', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})


		# x:119 y:516, x:358 y:275
		_sm_move_to_out_pose_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['pipes_pose'], output_keys=['pipes_out_pose'])

		with _sm_move_to_out_pose_3:
			# x:83 y:35
			OperatableStateMachine.add('Move_Pose_Out',
										CalculationState(calculation=lambda p: self.update_pose(p, self.pose_offset)),
										transitions={'done': 'Visualize_Pipes_Out_Pose'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pipes_pose', 'output_value': 'pipes_out_pose'})

			# x:85 y:307
			OperatableStateMachine.add('Move_To_Joints',
										MoveitToJointsState(move_group=move_group, joint_names=joints_arm_with_gripper, action_topic='/move_group'),
										transitions={'reached': 'finished', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Low, 'planning_failed': Autonomy.High, 'control_failed': Autonomy.High},
										remapping={'joint_config': 'joint_config'})

			# x:77 y:118
			OperatableStateMachine.add('Visualize_Pipes_Out_Pose',
										PublishPoseState(topic='/pipes/out_pose'),
										transitions={'done': 'Get_Joint_Target'},
										autonomy={'done': Autonomy.Low},
										remapping={'pose': 'pipes_out_pose'})

			# x:83 y:211
			OperatableStateMachine.add('Get_Joint_Target',
										CalculateIKState(move_group=move_group, ignore_collisions=False),
										transitions={'planned': 'Move_To_Joints', 'failed': 'failed'},
										autonomy={'planned': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'eef_pose': 'pipes_out_pose', 'joint_config': 'joint_config'})

			# x:205 y:395
			OperatableStateMachine.add('Correct_Gripper_Rotation',
										GripperRollState(rotation=GripperRollState.HORIZONTAL, duration=1.0),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.High})


		# x:363 y:42, x:153 y:375
		_sm_select_pipe_4 = OperatableStateMachine(outcomes=['finished', 'next'], input_keys=['index', 'object_pose', 'object_data'], output_keys=['index', 'pipe_pose'])

		with _sm_select_pipe_4:
			# x:98 y:78
			OperatableStateMachine.add('Increment_Index',
										CalculationState(calculation=lambda x: x+1),
										transitions={'done': 'Select_Pipe'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'index', 'output_value': 'index'})

			# x:102 y:178
			OperatableStateMachine.add('Select_Pipe',
										SelectPipePose(),
										transitions={'selected': 'next', 'out_of_range': 'finished'},
										autonomy={'selected': Autonomy.Off, 'out_of_range': Autonomy.Off},
										remapping={'object_pose': 'object_pose', 'object_data': 'object_data', 'pose_choice': 'index', 'pipe_pose': 'pipe_pose'})


		# x:272 y:264, x:55 y:574
		_sm_eef_to_detected_5 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['pipes_pose'], output_keys=['pipes_pose'])

		with _sm_eef_to_detected_5:
			# x:58 y:35
			OperatableStateMachine.add('Move_To_Out_Pose',
										_sm_move_to_out_pose_3,
										transitions={'finished': 'Decide_Move_Close', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pipes_pose': 'pipes_pose', 'pipes_out_pose': 'pipes_out_pose'})

			# x:568 y:38
			OperatableStateMachine.add('Move_To_Close_Pose',
										_sm_move_to_close_pose_2,
										transitions={'finished': 'Gripper_Action', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pipes_pose': 'pipes_pose'})

			# x:175 y:448
			OperatableStateMachine.add('Move_Back_Out',
										_sm_move_back_out_1,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pipes_out_pose': 'pipes_out_pose'})

			# x:326 y:34
			OperatableStateMachine.add('Decide_Move_Close',
										OperatorDecisionState(outcomes=['close', 'end'], hint="Move closer to pipe?", suggestion='close'),
										transitions={'close': 'Move_To_Close_Pose', 'end': 'finished'},
										autonomy={'close': Autonomy.High, 'end': Autonomy.Full})

			# x:626 y:219
			OperatableStateMachine.add('Gripper_Action',
										_sm_gripper_action_0,
										transitions={'finished': 'Decide_Move_Back', 'failed': 'failed', 'back': 'Move_Back_Out'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit, 'back': Autonomy.Inherit})

			# x:408 y:452
			OperatableStateMachine.add('Decide_Move_Back',
										OperatorDecisionState(outcomes=['move_back', 'end'], hint="Move back from pipe?", suggestion='move_back'),
										transitions={'move_back': 'Move_Back_Out', 'end': 'finished'},
										autonomy={'move_back': Autonomy.High, 'end': Autonomy.Full})


		# x:98 y:521, x:405 y:155
		_sm_get_pipes_6 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['object_pose'], output_keys=['object_pose', 'object_data'])

		with _sm_get_pipes_6:
			# x:51 y:28
			OperatableStateMachine.add('Get_Observation_Arm_Config',
										GetJointsFromSrdfState(config_name=obs_pose_side, srdf_file=srdf, move_group="", robot_name=""),
										transitions={'retrieved': 'Move_To_Observation_Pose', 'file_error': 'failed'},
										autonomy={'retrieved': Autonomy.Off, 'file_error': Autonomy.Off},
										remapping={'joint_values': 'joints_observation_pose'})

			# x:53 y:111
			OperatableStateMachine.add('Move_To_Observation_Pose',
										MoveitToJointsState(move_group="arm_group", joint_names=joints_arm_with_gripper[0:4], action_topic='/move_group'),
										transitions={'reached': 'Detect_Pipes', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Low, 'planning_failed': Autonomy.High, 'control_failed': Autonomy.High},
										remapping={'joint_config': 'joints_observation_pose'})

			# x:67 y:303
			OperatableStateMachine.add('Get_Pipes_Pose',
										MonitorPerceptState(required_support=0, percept_class='pipes', track_percepts=False),
										transitions={'detected': 'Visualize_Pipes_Pose'},
										autonomy={'detected': Autonomy.Off},
										remapping={'object_id': 'object_id', 'object_pose': 'object_pose', 'object_data': 'object_data'})

			# x:59 y:396
			OperatableStateMachine.add('Visualize_Pipes_Pose',
										PublishPoseState(topic='/pipes/center_pose'),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Low},
										remapping={'pose': 'object_pose'})

			# x:72 y:210
			OperatableStateMachine.add('Detect_Pipes',
										PipeDetectionState(max_attempts=100),
										transitions={'found': 'Get_Pipes_Pose', 'unknown': 'failed'},
										autonomy={'found': Autonomy.Low, 'unknown': Autonomy.High})



		with _state_machine:
			# x:94 y:72
			OperatableStateMachine.add('Get_Pipes',
										_sm_get_pipes_6,
										transitions={'finished': 'Select_Pipe', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'object_pose': 'object_pose', 'object_data': 'object_data'})

			# x:630 y:163
			OperatableStateMachine.add('EEF_To_Detected',
										_sm_eef_to_detected_5,
										transitions={'finished': 'Select_Pipe', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pipes_pose': 'pipes_pose'})

			# x:314 y:367
			OperatableStateMachine.add('Decide_Look_At',
										OperatorDecisionState(outcomes=['focus', 'skip'], hint="Should sensor head focus pipes?", suggestion='skip'),
										transitions={'focus': 'Look_At_Target', 'skip': 'EEF_To_Detected'},
										autonomy={'focus': Autonomy.Full, 'skip': Autonomy.High})

			# x:348 y:260
			OperatableStateMachine.add('Look_At_Target',
										LookAtWaypoint(),
										transitions={'reached': 'EEF_To_Detected', 'failed': 'failed'},
										autonomy={'reached': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'waypoint': 'pipes_pose'})

			# x:94 y:222
			OperatableStateMachine.add('Select_Pipe',
										_sm_select_pipe_4,
										transitions={'finished': 'finished', 'next': 'Decide_Look_At'},
										autonomy={'finished': Autonomy.Inherit, 'next': Autonomy.Inherit},
										remapping={'index': 'index', 'object_pose': 'object_pose', 'object_data': 'object_data', 'pipe_pose': 'pipes_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	def update_pose(self, pose, offset):
		new_pose = self._tf.transformPose('base_stabilized', pose)
		new_pose.pose.position.z = offset
		new_pose.pose.orientation.x = 0
		new_pose.pose.orientation.y = 0
		new_pose.pose.orientation.z = 0
		new_pose.pose.orientation.w = 1
		return new_pose
	# [/MANUAL_FUNC]
