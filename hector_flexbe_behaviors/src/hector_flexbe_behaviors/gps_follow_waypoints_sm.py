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
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jul 28 2017
@author: Gabriel Huettenberger
'''
class GPSfollowwaypointsSM(Behavior):
	'''
	Follow waypoints for outdoor challenge
	'''


	def __init__(self):
		super(GPSfollowwaypointsSM, self).__init__()
		self.name = 'GPS follow waypoints'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.waypoints = []

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:95
			OperatableStateMachine.add('get_waypoints',
										GetPathState(pathTopic='/path_to_follow'),
										transitions={'succeeded': 'follow_waypoints', 'failed': 'follow_waypoints'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoints2': 'waypoints2'})

			# x:254 y:81
			OperatableStateMachine.add('follow_waypoints',
										FollowWaypointsState(desired_speed=0.8, position_tolerance=0, angle_tolerance=3, rotate_to_goal=0, reexplore_time=5, use_planning=True),
										transitions={'reached': 'follow_waypoints', 'failed': 'follow_waypoints', 'reverse': 'reverse_waypoints'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off, 'reverse': Autonomy.Full},
										remapping={'waypoints': 'waypoints', 'waypoints2': 'waypoints2'})

			# x:259 y:305
			OperatableStateMachine.add('reverse_waypoints',
										ReverseListState(),
										transitions={'succeeded': 'follow_waypoints', 'failed': 'follow_waypoints'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoints': 'waypoints', 'waypoints2': 'waypoints2'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
