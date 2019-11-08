#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Sep 30 2019
@author: Gabriel Huettenberger
'''
class WRSPrototypeSM(Behavior):
	'''
	First prototype for WRS 2020
	'''


	def __init__(self):
		super(WRSPrototypeSM, self).__init__()
		self.name = 'WRS Prototype'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:365, x:130 y:365
		_sm_move_to_waypoint_0 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_move_to_waypoint_0:
			# x:30 y:40
			OperatableStateMachine.add('PLACEHOLDER',
										WaitState(wait_time=1),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})


		# x:30 y:365, x:130 y:365
		_sm_point_of_interest_1 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_point_of_interest_1:
			# x:63 y:48
			OperatableStateMachine.add('get_poi_waypoint',
										WaitState(wait_time=1),
										transitions={'done': 'move_to_waypoint'},
										autonomy={'done': Autonomy.Off})

			# x:255 y:34
			OperatableStateMachine.add('move_to_waypoint',
										_sm_move_to_waypoint_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})



		with _state_machine:
			# x:139 y:136
			OperatableStateMachine.add('get_next_poi',
										WaitState(wait_time=1),
										transitions={'done': 'Point of Interest'},
										autonomy={'done': Autonomy.Off})

			# x:363 y:141
			OperatableStateMachine.add('Point of Interest',
										_sm_point_of_interest_1,
										transitions={'finished': 'get_next_poi', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
