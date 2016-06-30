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
from hector_flexbe_states.monitor_percept_state import MonitorPerceptState
from flexbe_utility_states.publish_pose_state import PublishPoseState
from hector_flexbe_states.pipe_detection_state import PipeDetectionState
from flexbe_states.calculation_state import CalculationState
from hector_flexbe_states.calculate_ik_state import CalculateIKState
from hector_flexbe_states.moveit_execute_trajectory_state import MoveitExecuteTrajectoryState
from hector_flexbe_states.calculate_cartesian_path_state import CalculateCartesianPathState
from flexbe_states.operator_decision_state import OperatorDecisionState
from hector_flexbe_states.gripper_state import GripperState
from flexbe_states.log_state import LogState
from hector_flexbe_states.LookAtWaypoint import LookAtWaypoint
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from flexbe_core.proxy import ProxyTransformListener
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
		self.add_parameter('pose_offset', 0.3)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		self._tf = ProxyTransformListener().listener()
		# [/MANUAL_INIT]

		# Behavior comments:

		# O 36 553 
		# Run in full autonomy to skip looking, run in high to decide



	def create(self):
		joints_arm_with_gripper = ['arm_joint_%d'%i for i in range(5)]
		srdf = "hector_tracker_robot_moveit_config/config/taurob_tracker.srdf"
		percept_class = 'pipes'
		move_group = "arm_with_gripper_group"
		move_group_no_eef = "arm_group"
		# x:1128 y:547, x:472 y:46
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], output_keys=['pipes_pose'])
		_state_machine.userdata.pipes_pose = None
		_state_machine.userdata.object_type = percept_class
		_state_machine.userdata.test_pregrasp = [0, 1.57, 0.77, 0.38, 0]

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:569 y:352
		_sm_gripper_action_0 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_gripper_action_0:
			# x:168 y:53
			OperatableStateMachine.add('Decide_Close_Gripper',
										OperatorDecisionState(outcomes=['close_gripper', 'move_back'], hint="Grasp pipe?", suggestion='close_gripper'),
										transitions={'close_gripper': 'Close_Gripper', 'move_back': 'finished'},
										autonomy={'close_gripper': Autonomy.Full, 'move_back': Autonomy.Full})

			# x:221 y:173
			OperatableStateMachine.add('Close_Gripper',
										GripperState(action=0.8, duration=2.0),
										transitions={'done': 'Manipulate_Pipe', 'failed': 'failed'},
										autonomy={'done': Autonomy.Low, 'failed': Autonomy.High})

			# x:210 y:394
			OperatableStateMachine.add('Decide_Open_Gripper',
										OperatorDecisionState(outcomes=['open_gripper', 'move_back'], hint="Open gripper or pull pipe?", suggestion='open_gripper'),
										transitions={'open_gripper': 'Open_Gripper', 'move_back': 'finished'},
										autonomy={'open_gripper': Autonomy.High, 'move_back': Autonomy.Full})

			# x:206 y:288
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
										autonomy={'done': Autonomy.High, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})


		# x:230 y:517, x:550 y:229
		_sm_move_to_close_pose_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['pipes_pose'])

		with _sm_move_to_close_pose_2:
			# x:97 y:43
			OperatableStateMachine.add('Move_Pose_Cartesian',
										CalculationState(calculation=lambda p: self.update_pose(p, .05)),
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
										autonomy={'done': Autonomy.High, 'failed': Autonomy.High},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:121 y:260
			OperatableStateMachine.add('Get_Cartesian_Path',
										CalculateCartesianPathState(move_group=move_group, ignore_collisions=True),
										transitions={'planned': 'Execute_Cartesian_Path', 'failed': 'failed'},
										autonomy={'planned': Autonomy.Low, 'failed': Autonomy.High},
										remapping={'eef_pose': 'pipes_close_pose', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})


		# x:124 y:474, x:358 y:275
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
										autonomy={'reached': Autonomy.High, 'planning_failed': Autonomy.High, 'control_failed': Autonomy.High},
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


		# x:272 y:264, x:55 y:574
		_sm_eef_to_detected_4 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['pipes_pose'], output_keys=['pipes_pose'])

		with _sm_eef_to_detected_4:
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
										transitions={'finished': 'Decide_Move_Back', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:408 y:452
			OperatableStateMachine.add('Decide_Move_Back',
										OperatorDecisionState(outcomes=['move_back', 'end'], hint="Move back from pipe?", suggestion='move_back'),
										transitions={'end': 'finished', 'move_back': 'Move_Back_Out'},
										autonomy={'end': Autonomy.Full, 'move_back': Autonomy.High})


		# x:98 y:521, x:405 y:155
		_sm_get_pipes_5 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['pipes_pose'], output_keys=['pipes_pose'])

		with _sm_get_pipes_5:
			# x:51 y:28
			OperatableStateMachine.add('Get_Compact_Arm_Config',
										GetJointsFromSrdfState(config_name="compact_drive_pose", srdf_file=srdf, move_group="", robot_name=""),
										transitions={'retrieved': 'Move_To_Compact_Arm', 'file_error': 'failed'},
										autonomy={'retrieved': Autonomy.Off, 'file_error': Autonomy.Off},
										remapping={'joint_values': 'joints_compact_drive'})

			# x:53 y:111
			OperatableStateMachine.add('Move_To_Compact_Arm',
										MoveitToJointsState(move_group="arm_with_gripper_group", joint_names=joints_arm_with_gripper, action_topic='/move_group'),
										transitions={'reached': 'Detect_Pipes', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Low, 'planning_failed': Autonomy.High, 'control_failed': Autonomy.High},
										remapping={'joint_config': 'joints_compact_drive'})

			# x:67 y:303
			OperatableStateMachine.add('Get_Pipes_Pose',
										MonitorPerceptState(required_support=0, percept_class='pipes', track_percepts=False),
										transitions={'detected': 'Visualize_Pipes_Pose'},
										autonomy={'detected': Autonomy.Off},
										remapping={'object_id': 'object_id', 'object_pose': 'pipes_pose', 'object_data': 'object_data'})

			# x:59 y:396
			OperatableStateMachine.add('Visualize_Pipes_Pose',
										PublishPoseState(topic='/pipes/center_pose'),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Low},
										remapping={'pose': 'pipes_pose'})

			# x:72 y:210
			OperatableStateMachine.add('Detect_Pipes',
										PipeDetectionState(max_attempts=100),
										transitions={'found': 'Get_Pipes_Pose', 'unknown': 'failed'},
										autonomy={'found': Autonomy.Low, 'unknown': Autonomy.High})



		with _state_machine:
			# x:78 y:38
			OperatableStateMachine.add('Get_Pipes',
										_sm_get_pipes_5,
										transitions={'finished': 'Decide_Look_At', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pipes_pose': 'pipes_pose'})

			# x:578 y:319
			OperatableStateMachine.add('EEF_To_Detected',
										_sm_eef_to_detected_4,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pipes_pose': 'pipes_pose'})

			# x:492 y:454
			OperatableStateMachine.add('Move_To_Pregrasp',
										MoveitToJointsState(move_group="arm_group", joint_names=joints_arm_with_gripper[0:4], action_topic='/move_group'),
										transitions={'reached': 'EEF_To_Detected', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Low, 'planning_failed': Autonomy.High, 'control_failed': Autonomy.High},
										remapping={'joint_config': 'test_pregrasp'})

			# x:90 y:474
			OperatableStateMachine.add('Decide_Look_At',
										OperatorDecisionState(outcomes=['focus', 'skip'], hint="Should sensor head focus pipes?", suggestion='skip'),
										transitions={'skip': 'Move_To_Pregrasp', 'focus': 'Look_At_Target'},
										autonomy={'skip': Autonomy.High, 'focus': Autonomy.Full})

			# x:414 y:544
			OperatableStateMachine.add('Look_At_Target',
										LookAtWaypoint(),
										transitions={'reached': 'Move_To_Pregrasp', 'failed': 'failed'},
										autonomy={'reached': Autonomy.High, 'failed': Autonomy.Full},
										remapping={'waypoint': 'pipes_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	def update_pose(self, pose, offset):
		new_pose = self._tf.transformPose('base_link', pose)
		new_pose.pose.position.x -= offset
		return new_pose
	# [/MANUAL_FUNC]
