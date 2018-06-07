#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_argos_states.get_path_state import GetPathState
from hector_flexbe_states.get_waypoint_from_array_state import GetWaypointFromArrayState
from flexbe_argos_states.reverse_list_state import ReverseListState
from flexbe_argos_states.move_to_waypoint_state import MoveToWaypointState
from flexbe_states.wait_state import WaitState
from hector_flexbe_states.drive_straight_state import DriveStraightState
from hector_flexbe_states.has_reached_waypoint_state import HasReachedWaypointState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jun 05 2018
@author: Gabriel Hüttenberger
'''
class MANAlignSM(Behavior):
	'''
	Autonomous behavior for align task in Robocup
	'''


	def __init__(self):
		super(MANAlignSM, self).__init__()
		self.name = 'MAN Align'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:186 y:765, x:589 y:299
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.first_duration = 10
		_state_machine.userdata.position = 0
		_state_machine.userdata.waypoints = []

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:458, x:130 y:458, x:230 y:458, x:330 y:458, x:430 y:458, x:530 y:458
		_sm_traverse_plank_reverse_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['waypoint'], conditions=[
										('finished', [('drive_backward', 'reached')]),
										('failed', [('drive_backward', 'failed')]),
										('finished', [('has_reached_waypoint', 'reached')]),
										('failed', [('has_reached_waypoint', 'failed')])
										])

		with _sm_traverse_plank_reverse_0:
			# x:96 y:112
			OperatableStateMachine.add('drive_backward',
										DriveStraightState(desired_speed=0.2, duration=10, reverse=True, velocity_topic='/cmd_vel'),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:277 y:115
			OperatableStateMachine.add('has_reached_waypoint',
										HasReachedWaypointState(threshold=0.2, robot_pose_topic='/robot_pose'),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'waypoint'})


		# x:30 y:458, x:130 y:458, x:230 y:458, x:330 y:458, x:430 y:458, x:530 y:458
		_sm_traverse_plank_reverse_1 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['waypoint'], conditions=[
										('finished', [('drive_backwards', 'reached')]),
										('failed', [('drive_backwards', 'failed')]),
										('finished', [('has_reached_waypoint', 'reached')]),
										('failed', [('has_reached_waypoint', 'failed')])
										])

		with _sm_traverse_plank_reverse_1:
			# x:96 y:112
			OperatableStateMachine.add('drive_backwards',
										DriveStraightState(desired_speed=0.2, duration=10, reverse=False, velocity_topic='/cmd_vel'),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:277 y:115
			OperatableStateMachine.add('has_reached_waypoint',
										HasReachedWaypointState(threshold=0.2, robot_pose_topic='/robot_pose'),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'waypoint'})


		# x:30 y:458, x:130 y:458, x:230 y:458, x:330 y:458, x:430 y:458, x:530 y:458
		_sm_traverse_second_plank_2 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['waypoint'], conditions=[
										('finished', [('drive_forwad', 'reached')]),
										('failed', [('drive_forwad', 'failed')]),
										('finished', [('has_reached_waypoint', 'reached')]),
										('failed', [('has_reached_waypoint', 'failed')])
										])

		with _sm_traverse_second_plank_2:
			# x:96 y:112
			OperatableStateMachine.add('drive_forwad',
										DriveStraightState(desired_speed=0.2, duration=10, reverse=False, velocity_topic='/cmd_vel'),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:277 y:115
			OperatableStateMachine.add('has_reached_waypoint',
										HasReachedWaypointState(threshold=0.2, robot_pose_topic='/robot_pose'),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'waypoint'})


		# x:30 y:458, x:130 y:458, x:230 y:458, x:330 y:458, x:430 y:458, x:530 y:458
		_sm_traverse_first_plank_3 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['waypoint'], conditions=[
										('finished', [('Drive_forward', 'reached')]),
										('failed', [('Drive_forward', 'failed')]),
										('finished', [('has_reached_waypoint', 'reached')]),
										('failed', [('has_reached_waypoint', 'failed')])
										])

		with _sm_traverse_first_plank_3:
			# x:96 y:112
			OperatableStateMachine.add('Drive_forward',
										DriveStraightState(desired_speed=0.2, duration=10, reverse=False, velocity_topic='/cmd_vel'),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:277 y:115
			OperatableStateMachine.add('has_reached_waypoint',
										HasReachedWaypointState(threshold=0.2, robot_pose_topic='/robot_pose'),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'waypoint'})


		# x:30 y:458, x:130 y:458
		_sm_first_plank_reverse_4 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['waypoint'])

		with _sm_first_plank_reverse_4:
			# x:192 y:92
			OperatableStateMachine.add('PLACEHOLDER1',
										WaitState(wait_time=1),
										transitions={'done': 'Traverse_Plank_Reverse'},
										autonomy={'done': Autonomy.Off})

			# x:294 y:345
			OperatableStateMachine.add('stop_robot',
										DriveStraightState(desired_speed=0, duration=1, reverse=False, velocity_topic='/cmd_vel'),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:544 y:87
			OperatableStateMachine.add('Traverse_Plank_Reverse',
										_sm_traverse_plank_reverse_0,
										transitions={'finished': 'stop_robot', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'waypoint': 'waypoint'})


		# x:309 y:428, x:184 y:272
		_sm_second_plank_reverse_5 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['waypoint2'])

		with _sm_second_plank_reverse_5:
			# x:46 y:115
			OperatableStateMachine.add('Traverse_Plank_Reverse',
										_sm_traverse_plank_reverse_1,
										transitions={'finished': 'stop_robot', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'waypoint': 'waypoint2'})

			# x:337 y:210
			OperatableStateMachine.add('stop_robot',
										DriveStraightState(desired_speed=0, duration=1, reverse=False, velocity_topic='/cmd_vel'),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})


		# x:30 y:458, x:306 y:196
		_sm_second_plank_6 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['waypoint'])

		with _sm_second_plank_6:
			# x:195 y:57
			OperatableStateMachine.add('PLACEHOLDER2',
										WaitState(wait_time=1),
										transitions={'done': 'Traverse_Second_Plank'},
										autonomy={'done': Autonomy.Off})

			# x:315 y:343
			OperatableStateMachine.add('stop_robot',
										DriveStraightState(desired_speed=0, duration=1, reverse=False, velocity_topic='/cmd_vel'),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:451 y:103
			OperatableStateMachine.add('Traverse_Second_Plank',
										_sm_traverse_second_plank_2,
										transitions={'finished': 'stop_robot', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'waypoint': 'waypoint'})


		# x:30 y:458, x:360 y:206
		_sm_first_plank_7 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['waypoint'])

		with _sm_first_plank_7:
			# x:192 y:92
			OperatableStateMachine.add('PLACEHOLDER1',
										WaitState(wait_time=1),
										transitions={'done': 'Traverse_First_Plank'},
										autonomy={'done': Autonomy.Off})

			# x:294 y:345
			OperatableStateMachine.add('stop_robot',
										DriveStraightState(desired_speed=0, duration=1, reverse=False, velocity_topic='/cmd_vel'),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off})

			# x:544 y:87
			OperatableStateMachine.add('Traverse_First_Plank',
										_sm_traverse_first_plank_3,
										transitions={'finished': 'stop_robot', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'waypoint': 'waypoint'})


		# x:30 y:458, x:463 y:298
		_sm_initialize_8 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['waypoints'], output_keys=['waypoint1', 'waypoint2', 'waypoint3', 'waypoint4'])

		with _sm_initialize_8:
			# x:155 y:98
			OperatableStateMachine.add('get_pseudo_waypoints',
										GetPathState(pathTopic='/path_to_follow'),
										transitions={'succeeded': 'reverse_waypoints', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoints2': 'waypoints2'})

			# x:695 y:105
			OperatableStateMachine.add('get_first_waypoint',
										GetWaypointFromArrayState(position=0),
										transitions={'reached': 'get_second_waypoint', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoints': 'waypoints', 'waypoint': 'waypoint1'})

			# x:711 y:257
			OperatableStateMachine.add('get_second_waypoint',
										GetWaypointFromArrayState(position=1),
										transitions={'reached': 'get_third_waypoint', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoints': 'waypoints', 'waypoint': 'waypoint2'})

			# x:384 y:104
			OperatableStateMachine.add('reverse_waypoints',
										ReverseListState(),
										transitions={'succeeded': 'get_first_waypoint', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoints': 'waypoints', 'waypoints2': 'waypoints2'})

			# x:685 y:465
			OperatableStateMachine.add('get_third_waypoint',
										GetWaypointFromArrayState(position=2),
										transitions={'reached': 'get_fourth_waypoint', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoints': 'waypoints', 'waypoint': 'waypoint3'})

			# x:363 y:489
			OperatableStateMachine.add('get_fourth_waypoint',
										GetWaypointFromArrayState(position=3),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoints': 'waypoints', 'waypoint': 'waypoint4'})



		with _state_machine:
			# x:138 y:64
			OperatableStateMachine.add('Initialize',
										_sm_initialize_8,
										transitions={'finished': 'First_Plank', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'waypoints': 'waypoints', 'waypoint1': 'waypoint1', 'waypoint2': 'waypoint2', 'waypoint3': 'waypoint3', 'waypoint4': 'waypoint4'})

			# x:343 y:505
			OperatableStateMachine.add('move_to_first_waypoint',
										MoveToWaypointState(desired_speed=0.2, position_tolerance=0.1, angle_tolerance=3, rotate_to_goal=False),
										transitions={'reached': 'First_Plank_Reverse', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'waypoint1'})

			# x:376 y:72
			OperatableStateMachine.add('First_Plank',
										_sm_first_plank_7,
										transitions={'finished': 'move_to_second_waypoint', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'waypoint': 'waypoint1'})

			# x:756 y:291
			OperatableStateMachine.add('Second_Plank',
										_sm_second_plank_6,
										transitions={'finished': 'Second_Plank_Reverse', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'waypoint': 'waypoint3'})

			# x:743 y:77
			OperatableStateMachine.add('move_to_second_waypoint',
										MoveToWaypointState(desired_speed=0.2, position_tolerance=0.1, angle_tolerance=3, rotate_to_goal=False),
										transitions={'reached': 'Second_Plank', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'waypoint2'})

			# x:737 y:501
			OperatableStateMachine.add('Second_Plank_Reverse',
										_sm_second_plank_reverse_5,
										transitions={'finished': 'move_to_first_waypoint', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'waypoint2': 'waypoint2'})

			# x:311 y:293
			OperatableStateMachine.add('First_Plank_Reverse',
										_sm_first_plank_reverse_4,
										transitions={'finished': 'First_Plank', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'waypoint': 'waypoint4'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
