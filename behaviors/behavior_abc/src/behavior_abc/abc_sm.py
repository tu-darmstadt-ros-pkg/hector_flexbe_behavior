#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_abc')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hector_flexbe_states.Wait_Exploration import Wait_Exploration
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu May 19 2016
@author: ghi
'''
class abcSM(Behavior):
	'''
	def
	'''


	def __init__(self):
		super(abcSM, self).__init__()
		self.name = 'abc'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.goalId = ' '

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('wait',
										Wait_Exploration(),
										transitions={'aborted': 'wait', 'succeeded': 'wait', 'waiting': 'wait', 'getVictim': 'wait'},
										autonomy={'aborted': Autonomy.Off, 'succeeded': Autonomy.Off, 'waiting': Autonomy.Off, 'getVictim': Autonomy.Off},
										remapping={'goalId': 'goalId'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
