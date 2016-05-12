#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_behavior_exploration')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hector_flexbe_states.START_Exploration_Transform import START_Exploration_Transform
from hector_flexbe_states.Error_Exploration import Error_Exploration
from hector_flexbe_states.Wait_Exploration import Wait_Exploration
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Nov 19 2015
@author: Elisa und Gabriel
'''
class behavior_explorationSM(Behavior):
	'''
	first try
	'''


	def __init__(self):
		super(behavior_explorationSM, self).__init__()
		self.name = 'behavior_exploration'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:916 y:133
		_state_machine = OperatableStateMachine(outcomes=['finished'])
		_state_machine.userdata.goalId = 'abcd'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:50 y:39
			OperatableStateMachine.add('StartExploration',
										START_Exploration_Transform(),
										transitions={'succeeded': 'Wait'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'goalId': 'goalId'})

			# x:280 y:158
			OperatableStateMachine.add('Error',
										Error_Exploration(),
										transitions={'restart': 'StartExploration'},
										autonomy={'restart': Autonomy.Off})

			# x:491 y:39
			OperatableStateMachine.add('Wait',
										Wait_Exploration(),
										transitions={'aborted': 'Error', 'succeeded': 'Error', 'waiting': 'Wait', 'getVictim': 'finished'},
										autonomy={'aborted': Autonomy.Off, 'succeeded': Autonomy.Off, 'waiting': Autonomy.Off, 'getVictim': Autonomy.Off},
										remapping={'goalId': 'goalId'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
