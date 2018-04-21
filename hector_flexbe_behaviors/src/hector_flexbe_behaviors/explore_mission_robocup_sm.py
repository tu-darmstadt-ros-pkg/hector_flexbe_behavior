#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_argos_states.explore import Explore
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jul 20 2017
@author: Gabriel Huettenberger
'''
class ExploreMissionRobocupSM(Behavior):
	'''
	Default behavior for exploration based Robocup missions
	'''


	def __init__(self):
		super(ExploreMissionRobocupSM, self).__init__()
		self.name = 'Explore Mission Robocup'

		# parameters of this behavior
		self.add_parameter('speed', 0.2)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.speed = self.speed
		_state_machine.userdata.reexplore_time = 5

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:91 y:128
			OperatableStateMachine.add('Start Exploration',
										Explore(),
										transitions={'succeeded': 'Wait', 'failed': 'Wait'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'speed': 'speed', 'reexplore_time': 'reexplore_time'})

			# x:341 y:63
			OperatableStateMachine.add('Wait',
										WaitState(wait_time=1),
										transitions={'done': 'Start Exploration'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
