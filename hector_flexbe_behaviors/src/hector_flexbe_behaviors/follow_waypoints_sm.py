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
from flexbe_argos_states.follow_waypoints_state import FollowWaypointsState
from flexbe_argos_states.reverse_list_state import ReverseListState
from hector_flexbe_states.get_recovery_info_state import GetRecoveryInfoState
from flexbe_argos_states.move_to_waypoint_state import MoveToWaypointState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jul 28 2017
@author: Gabriel Huettenberger
'''
class FollowwaypointsSM(Behavior):
	'''
	Follows a set of waypoints
	'''


	def __init__(self):
		super(FollowwaypointsSM, self).__init__()
		self.name = 'Follow waypoints'

		# parameters of this behavior
		self.add_parameter('speed', 0.2)
		self.add_parameter('use_planning', True)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:237 y:360
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.waypoints = []

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('get_path',
										GetPathState(pathTopic='/path_to_follow'),
										transitions={'succeeded': 'reverse_waypoints', 'failed': 'get_path'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoints2': 'waypoints2'})

			# x:627 y:66
			OperatableStateMachine.add('follow_waypoints',
										FollowWaypointsState(desired_speed=self.speed, position_tolerance=0.0, angle_tolerance=3, rotate_to_goal=False, reexplore_time=5, reverse_allowed=True, reverse_forced=False, use_planning=self.use_planning),
										transitions={'reached': 'follow_waypoints', 'failed': 'failed', 'reverse': 'reverse_waypoints', 'stuck': 'stuck_recovery'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off, 'reverse': Autonomy.Off, 'stuck': Autonomy.Off},
										remapping={'waypoints': 'waypoints', 'waypoints2': 'waypoints2'})

			# x:276 y:55
			OperatableStateMachine.add('reverse_waypoints',
										ReverseListState(),
										transitions={'succeeded': 'follow_waypoints', 'failed': 'follow_waypoints'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoints': 'waypoints', 'waypoints2': 'waypoints2'})

			# x:571 y:257
			OperatableStateMachine.add('stuck_recovery',
										GetRecoveryInfoState(service_topic='/trajectory_recovery_info'),
										transitions={'success': 'stuck_behavior', 'failed': 'follow_waypoints'},
										autonomy={'success': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'waypoint'})

			# x:746 y:258
			OperatableStateMachine.add('stuck_behavior',
										MoveToWaypointState(desired_speed=self.speed, position_tolerance=0, angle_tolerance=3, rotate_to_goal=False, reverse_allowed=True),
										transitions={'reached': 'follow_waypoints', 'failed': 'follow_waypoints'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'waypoint'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
