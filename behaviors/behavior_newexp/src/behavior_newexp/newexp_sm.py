#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_newexp')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hector_flexbe_states.START_Exploration_Transform import START_Exploration_Transform
from hector_flexbe_states.Error_Exploration import Error_Exploration
from hector_flexbe_states.Pause_Exploration import PauseExploration
from hector_flexbe_states.Wait_Exploration1 import Wait_Exploration1
from hector_flexbe_states.Object_Found_Exploration import Object_Found_Exploration
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat May 07 2016
@author: Gabriel
'''
class NewExpSM(Behavior):
	'''
	version 0.1
	'''


	def __init__(self):
		super(NewExpSM, self).__init__()
		self.name = 'NewExp'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:41 y:562, x:172 y:564
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['goalId'])
		_state_machine.userdata.goalId = 1

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Start',
										START_Exploration_Transform(),
										transitions={'succeeded': 'Wait'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'goalId': 'goalId'})

			# x:71 y:351
			OperatableStateMachine.add('Error',
										Error_Exploration(),
										transitions={'restart': 'Start'},
										autonomy={'restart': Autonomy.Off})

			# x:402 y:396
			OperatableStateMachine.add('Pause',
										PauseExploration(),
										transitions={'continue': 'Start', 'pause': 'Pause'},
										autonomy={'continue': Autonomy.Off, 'pause': Autonomy.Off})

			# x:400 y:202
			OperatableStateMachine.add('Wait',
										Wait_Exploration1(),
										transitions={'error': 'Error', 'waiting': 'Wait', 'victim': 'Pause', 'pause': 'Pause'},
										autonomy={'error': Autonomy.Off, 'waiting': Autonomy.Off, 'victim': Autonomy.Off, 'pause': Autonomy.Off},
										remapping={'goalId': 'goalId', 'object': 'object'})

			# x:652 y:298
			OperatableStateMachine.add('Object_Found',
										Object_Found_Exploration(),
										transitions={'unknown': 'Object_Found', 'qr_code': 'Pause', 'victim': 'Pause'},
										autonomy={'unknown': Autonomy.Off, 'qr_code': Autonomy.Off, 'victim': Autonomy.Off},
										remapping={'object': 'object', 'pose': 'pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
