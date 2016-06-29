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
from hector_flexbe_states.LookAtWaypoint import LookAtWaypoint
from hector_flexbe_states.pipe_detection_state import PipeDetectionState
from flexbe_states.calculation_state import CalculationState
from hector_flexbe_states.calculate_ik_state import CalculateIKState
from hector_flexbe_states.calculate_cartesian_path_state import CalculateCartesianPathState
from hector_flexbe_states.moveit_execute_trajectory_state import MoveitExecuteTrajectoryState
from hector_flexbe_states.gripper_state import GripperState
from flexbe_states.wait_state import WaitState
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



	def create(self):
		joints_arm_with_gripper = ['arm_joint_%d'%i for i in range(5)]
		srdf = "hector_tracker_robot_moveit_config/config/taurob_tracker.srdf"
		percept_class = 'pipes'
		move_group = "arm_with_gripper_group"
		move_group_no_eef = "arm_group"
		# x:1128 y:547, x:383 y:40
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], output_keys=['pipes_pose'])
		_state_machine.userdata.pipes_pose = None
		_state_machine.userdata.object_type = percept_class
		_state_machine.userdata.test_pregrasp = [0, 1.57, 0.77, 0.38, 0]

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:748 y:480, x:130 y:365
		_sm_eef_to_detected_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['pipes_pose'], output_keys=['pipes_pose'])

		with _sm_eef_to_detected_0:
			# x:141 y:28
			OperatableStateMachine.add('Move_Pose_Out',
										CalculationState(calculation=lambda p: self.update_pose(p, self.pose_offset)),
										transitions={'done': 'Visualize_Pipes_Pose_2'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pipes_pose', 'output_value': 'pipes_pose_shifted'})

			# x:319 y:310
			OperatableStateMachine.add('Move_To_Joints',
										MoveitToJointsState(move_group=move_group, joint_names=joints_arm_with_gripper, action_topic='/move_group'),
										transitions={'reached': 'Move_Pose_Cartesian', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'joint_config'})

			# x:217 y:133
			OperatableStateMachine.add('Visualize_Pipes_Pose_2',
										PublishPoseState(topic='/debug/pose2'),
										transitions={'done': 'Get_Joint_Target'},
										autonomy={'done': Autonomy.Off},
										remapping={'pose': 'pipes_pose_shifted'})

			# x:259 y:200
			OperatableStateMachine.add('Get_Joint_Target',
										CalculateIKState(move_group=move_group, ignore_collisions=False),
										transitions={'planned': 'Move_To_Joints', 'failed': 'failed'},
										autonomy={'planned': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'eef_pose': 'pipes_pose_shifted', 'joint_config': 'joint_config'})

			# x:511 y:346
			OperatableStateMachine.add('Get_Cartesian_Path',
										CalculateCartesianPathState(move_group=move_group, ignore_collisions=True),
										transitions={'planned': 'Execute_Cartesian_Path', 'failed': 'failed'},
										autonomy={'planned': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'eef_pose': 'pipes_pose_cartesian', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:526 y:171
			OperatableStateMachine.add('Move_Pose_Cartesian',
										CalculationState(calculation=lambda p: self.update_pose(p, .05)),
										transitions={'done': 'Visualize_Pipes_Pose_3'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'pipes_pose', 'output_value': 'pipes_pose_cartesian'})

			# x:617 y:247
			OperatableStateMachine.add('Visualize_Pipes_Pose_3',
										PublishPoseState(topic='/debug/pose3'),
										transitions={'done': 'Get_Cartesian_Path'},
										autonomy={'done': Autonomy.Off},
										remapping={'pose': 'pipes_pose_cartesian'})

			# x:509 y:432
			OperatableStateMachine.add('Execute_Cartesian_Path',
										MoveitExecuteTrajectoryState(),
										transitions={'done': 'Close_Gripper', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:262 y:631
			OperatableStateMachine.add('Get_Cartesian_Path_Back',
										CalculateCartesianPathState(move_group=move_group, ignore_collisions=True),
										transitions={'planned': 'Execute_Cartesian_Path_Back', 'failed': 'failed'},
										autonomy={'planned': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'eef_pose': 'pipes_pose_shifted', 'joint_trajectory': 'joint_trajectory', 'plan_fraction': 'plan_fraction'})

			# x:694 y:658
			OperatableStateMachine.add('Execute_Cartesian_Path_Back',
										MoveitExecuteTrajectoryState(),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'joint_trajectory': 'joint_trajectory'})

			# x:559 y:519
			OperatableStateMachine.add('Close_Gripper',
										GripperState(action=0.8, duration=2.0),
										transitions={'done': 'Wait', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:303 y:521
			OperatableStateMachine.add('Open_Gripper',
										GripperState(action=GripperState.OPEN, duration=1.0),
										transitions={'done': 'Get_Cartesian_Path_Back', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off})

			# x:443 y:544
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=1),
										transitions={'done': 'Open_Gripper'},
										autonomy={'done': Autonomy.Off})



		with _state_machine:
			# x:74 y:28
			OperatableStateMachine.add('Get_Compact_Arm_Config',
										GetJointsFromSrdfState(config_name="compact_drive_pose", srdf_file=srdf, move_group="", robot_name=""),
										transitions={'retrieved': 'Move_To_Compact_Arm', 'file_error': 'failed'},
										autonomy={'retrieved': Autonomy.Off, 'file_error': Autonomy.Off},
										remapping={'joint_values': 'joints_compact_drive'})

			# x:64 y:130
			OperatableStateMachine.add('Move_To_Compact_Arm',
										MoveitToJointsState(move_group="arm_with_gripper_group", joint_names=joints_arm_with_gripper, action_topic='/move_group'),
										transitions={'reached': 'Detect_Pipes', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'joints_compact_drive'})

			# x:88 y:328
			OperatableStateMachine.add('Get_Pipes_Pose',
										MonitorPerceptState(required_support=0, percept_class='pipes', track_percepts=False),
										transitions={'detected': 'Visualize_Pipes_Pose'},
										autonomy={'detected': Autonomy.Off},
										remapping={'object_id': 'object_id', 'object_pose': 'pipes_pose', 'object_data': 'object_data'})

			# x:285 y:313
			OperatableStateMachine.add('Visualize_Pipes_Pose',
										PublishPoseState(topic='/debug/pose'),
										transitions={'done': 'Look_At_Target'},
										autonomy={'done': Autonomy.Off},
										remapping={'pose': 'pipes_pose'})

			# x:445 y:225
			OperatableStateMachine.add('Look_At_Target',
										LookAtWaypoint(),
										transitions={'reached': 'Move_To_Pregrasp', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'pipes_pose'})

			# x:81 y:250
			OperatableStateMachine.add('Detect_Pipes',
										PipeDetectionState(max_attempts=100),
										transitions={'found': 'Get_Pipes_Pose', 'unknown': 'failed'},
										autonomy={'found': Autonomy.Off, 'unknown': Autonomy.Off})

			# x:970 y:83
			OperatableStateMachine.add('EEF_To_Detected',
										_sm_eef_to_detected_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pipes_pose': 'pipes_pose'})

			# x:645 y:145
			OperatableStateMachine.add('Move_To_Pregrasp',
										MoveitToJointsState(move_group="arm_group", joint_names=joints_arm_with_gripper[0:4], action_topic='/move_group'),
										transitions={'reached': 'EEF_To_Detected', 'planning_failed': 'failed', 'control_failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'planning_failed': Autonomy.Off, 'control_failed': Autonomy.Off},
										remapping={'joint_config': 'test_pregrasp'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	def update_pose(self, pose, offset):
		new_pose = self._tf.transformPose('base_link', pose)
		new_pose.pose.position.x -= offset
		return new_pose
	# [/MANUAL_FUNC]
