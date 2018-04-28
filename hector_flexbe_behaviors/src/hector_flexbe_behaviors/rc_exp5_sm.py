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
from hector_flexbe_states.explore_to_waypoint_state import ExploreToWaypointState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Apr 27 2018
@author: Gabriel Huettenberger
'''
class rc_exp5SM(Behavior):
	'''
	Custom behavior for exp5 mission of robocup german open 2018
	'''


	def __init__(self):
		super(rc_exp5SM, self).__init__()
		self.name = 'rc_exp5'

		# parameters of this behavior
		self.add_parameter('speed', 0.2)
		self.add_parameter('reexplore_time', 3)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:463 y:164, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.reexplore = self.reexplore_time

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:102
			OperatableStateMachine.add('get_path',
										GetPathState(pathTopic='/path_to_follow'),
										transitions={'succeeded': 'Explore_to_waypoint', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoints2': 'waypoint'})

			# x:228 y:71
			OperatableStateMachine.add('Explore_to_waypoint',
										ExploreToWaypointState(desired_speed=self.speed, position_tolerance=0, angle_tolerance=3, rotate_to_goal=0, reexplore_time=self.reexplore_time),
										transitions={'reached': 'finished', 'failed': 'failed'},
										autonomy={'reached': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'waypoint': 'waypoint'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
