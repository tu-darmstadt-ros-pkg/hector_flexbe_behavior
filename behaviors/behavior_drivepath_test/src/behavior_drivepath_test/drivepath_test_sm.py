#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_drivepath_test')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hector_flexbe_states.drivepath_test import DrivepathTest
from hector_flexbe_states.drivepath_test_new import DrivepathTestNew
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jun 28 2016
@author: def
'''
class DrivepathTestSM(Behavior):
	'''
	abc
	'''


	def __init__(self):
		super(DrivepathTestSM, self).__init__()
		self.name = 'Drivepath Test'

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


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Test',
										DrivepathTest(),
										transitions={'reached': 'Test2', 'failed': 'failed'},
										autonomy={'reached': Autonomy.High, 'failed': Autonomy.Off})

			# x:221 y:37
			OperatableStateMachine.add('Test2',
										DrivepathTestNew(),
										transitions={'reached': 'Test', 'failed': 'failed'},
										autonomy={'reached': Autonomy.High, 'failed': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
