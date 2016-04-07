#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_exploration2')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, Logger
from hector_flexbe_states.START_Exploration_Transform import START_Exploration_Transform
from hector_flexbe_states.Wait_Exploration import Wait_Exploration
from hector_flexbe_states.Error_Exploration import Error_Exploration
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Fri Jan 15 2016
@author: Elisa
'''
class Exploration2SM(Behavior):
	'''
	exploration nr.2
	'''


	def __init__(self):
		super(Exploration2SM, self).__init__()
		self.name = 'Exploration2'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:322
		_state_machine = OperatableStateMachine(outcomes=['finished'])
		_state_machine.userdata.goalId = 'abc'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('StartExploration',
										START_Exploration_Transform(),
										transitions={'succeeded': 'Wait'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'goalId': 'goalId'})

			# x:308 y:109
			OperatableStateMachine.add('Wait',
										Wait_Exploration(),
										transitions={'succeeded': 'Error', 'aborted': 'Error', 'waiting': 'Wait'},
										autonomy={'succeeded': Autonomy.Off, 'aborted': Autonomy.Off, 'waiting': Autonomy.Off},
										remapping={'goalId': 'goalId'})

			# x:33 y:155
			OperatableStateMachine.add('Error',
										Error_Exploration(),
										transitions={'restart': 'finished'},
										autonomy={'restart': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
