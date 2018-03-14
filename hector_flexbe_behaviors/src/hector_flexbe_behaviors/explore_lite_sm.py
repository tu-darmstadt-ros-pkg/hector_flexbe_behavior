#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_argos_states.cam_sweep_state_enr import CamSweepStateEnr
from flexbe_argos_states.explore import Explore
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jun 13 2017
@author: Gabriel Huettenberger
'''
class ExploreLiteSM(Behavior):
	'''
	Explore behavior using move_base_lite
	'''


	def __init__(self):
		super(ExploreLiteSM, self).__init__()
		self.name = 'Explore Lite'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		sweep_velocity = 0.2
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.speed = 0.2
		_state_machine.userdata.reexplore_time = 5

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:110
			OperatableStateMachine.add('CamSweepStart',
										CamSweepStateEnr(sweep_velocity=0.1, timeout=0),
										transitions={'done': 'Explore'},
										autonomy={'done': Autonomy.Off})

			# x:201 y:55
			OperatableStateMachine.add('Explore',
										Explore(),
										transitions={'succeeded': 'finished', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'speed': 'speed', 'reexplore_time': 'reexplore_time'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
