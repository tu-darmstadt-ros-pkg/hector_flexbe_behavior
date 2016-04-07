#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_wait_test')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, Logger
from hector_flexbe_states.Wait_Exploration import Wait_Exploration
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jan 21 2016
@author: Elisa
'''
class Wait_TestSM(Behavior):
	'''
	Test
	'''


	def __init__(self):
		super(Wait_TestSM, self).__init__()
		self.name = 'Wait_Test'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:322, x:130 y:322
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.goalId = 'abcd'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Wait',
										Wait_Exploration(),
										transitions={'succeeded': 'finished', 'aborted': 'failed', 'waiting': 'Wait'},
										autonomy={'succeeded': Autonomy.Off, 'aborted': Autonomy.Off, 'waiting': Autonomy.Off},
										remapping={'goalId': 'goalId'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
