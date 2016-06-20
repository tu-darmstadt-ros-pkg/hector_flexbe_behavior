#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_exploration')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hector_flexbe_states.LookAtPattern import LookAtPattern
from hector_flexbe_states.Error_Exploration import Error_Exploration
from hector_flexbe_states.start_exploration import StartExploration
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat May 07 2016
@author: Gabriel
'''
class ExplorationSM(Behavior):
	'''
	version 0.1
	'''


	def __init__(self):
		super(ExplorationSM, self).__init__()
		self.name = 'Exploration'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:41 y:562, x:172 y:564
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.lookAround = 'look_around'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:78 y:51
			OperatableStateMachine.add('Look_Around',
										LookAtPattern(),
										transitions={'succeeded': 'Exploration', 'failed': 'Exploration'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pattern': 'lookAround'})

			# x:226 y:353
			OperatableStateMachine.add('Error',
										Error_Exploration(),
										transitions={'restart': 'Exploration'},
										autonomy={'restart': Autonomy.High})

			# x:302 y:53
			OperatableStateMachine.add('Exploration',
										StartExploration(),
										transitions={'succeeded': 'finished', 'failed': 'Error'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
