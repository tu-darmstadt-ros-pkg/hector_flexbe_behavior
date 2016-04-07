#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_hectorbehavior')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, Logger
from hector_flexbe_states.behavior2 import behavior2
from hector_flexbe_states.START_Exploration_Transform import START_Exploration_Transform
from hector_flexbe_states.Error_Exploration import Error_Exploration
from hector_flexbe_states.Wait_Exploration import Wait_Exploration
from hector_flexbe_states.Wait_StartHack import Wait_StartHack
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jan 12 2016
@author: Elisa und Gabriel
'''
class HectorBehaviorSM(Behavior):
	'''
	behavior demonstration for presentation
	'''


	def __init__(self):
		super(HectorBehaviorSM, self).__init__()
		self.name = 'HectorBehavior'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1083 y:240
		_state_machine = OperatableStateMachine(outcomes=['done'])
		_state_machine.userdata.goalId = 'abcd'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:322
		_sm_explorevictim_0 = OperatableStateMachine(outcomes=['done'])

		with _sm_explorevictim_0:
			# x:118 y:91
			OperatableStateMachine.add('Wait',
										Wait_StartHack(),
										transitions={'true': 'done', 'false': 'done'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off})


		# x:30 y:322
		_sm_starthack_1 = OperatableStateMachine(outcomes=['done'])

		with _sm_starthack_1:
			# x:118 y:91
			OperatableStateMachine.add('Wait',
										Wait_StartHack(),
										transitions={'true': 'done', 'false': 'done'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off})


		# x:30 y:322
		_sm_lookat_2 = OperatableStateMachine(outcomes=['done'])

		with _sm_lookat_2:
			# x:118 y:91
			OperatableStateMachine.add('Wait',
										Wait_StartHack(),
										transitions={'true': 'done', 'false': 'done'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off})


		# x:30 y:322
		_sm_cancel_3 = OperatableStateMachine(outcomes=['done'])

		with _sm_cancel_3:
			# x:118 y:91
			OperatableStateMachine.add('Wait',
										Wait_StartHack(),
										transitions={'true': 'done', 'false': 'done'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off})


		# x:30 y:322
		_sm_approachvictim_4 = OperatableStateMachine(outcomes=['finished'])

		with _sm_approachvictim_4:
			# x:118 y:91
			OperatableStateMachine.add('Wait',
										Wait_StartHack(),
										transitions={'true': 'finished', 'false': 'finished'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off})


		# x:30 y:322
		_sm_getclosertovictim_5 = OperatableStateMachine(outcomes=['finished'])

		with _sm_getclosertovictim_5:
			# x:118 y:91
			OperatableStateMachine.add('Wait',
										Wait_StartHack(),
										transitions={'true': 'finished', 'false': 'finished'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off})


		# x:30 y:373
		_sm_exploration_6 = OperatableStateMachine(outcomes=['finished'], input_keys=['goalId'])

		with _sm_exploration_6:
			# x:218 y:66
			OperatableStateMachine.add('StartExploration',
										START_Exploration_Transform(),
										transitions={'succeeded': 'Wait'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'goalId': 'goalId'})

			# x:49 y:228
			OperatableStateMachine.add('Error',
										Error_Exploration(),
										transitions={'restart': 'StartExploration'},
										autonomy={'restart': Autonomy.Off})

			# x:400 y:228
			OperatableStateMachine.add('Wait',
										Wait_Exploration(),
										transitions={'succeeded': 'Error', 'aborted': 'Error', 'waiting': 'Wait'},
										autonomy={'succeeded': Autonomy.Low, 'aborted': Autonomy.Low, 'waiting': Autonomy.Low},
										remapping={'goalId': 'goalId'})



		with _state_machine:
			# x:79 y:228
			OperatableStateMachine.add('Initialisation',
										behavior2(),
										transitions={'APPROACH_VICTIM': 'ApproachVictim', 'EXPLORE': 'Exploration', 'EXPLORE_VICTIM_HYPOTHESIS': 'ExploreVictim', 'GO_TO': 'GetCloserToVictim', 'START': 'StartHack', 'LOOK_AT': 'LookAt', 'CANCEL': 'Cancel'},
										autonomy={'APPROACH_VICTIM': Autonomy.Off, 'EXPLORE': Autonomy.Low, 'EXPLORE_VICTIM_HYPOTHESIS': Autonomy.Off, 'GO_TO': Autonomy.Off, 'START': Autonomy.Off, 'LOOK_AT': Autonomy.Off, 'CANCEL': Autonomy.Off})

			# x:444 y:14
			OperatableStateMachine.add('Exploration',
										_sm_exploration_6,
										transitions={'finished': 'done'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'goalId': 'goalId'})

			# x:535 y:286
			OperatableStateMachine.add('GetCloserToVictim',
										_sm_getclosertovictim_5,
										transitions={'finished': 'done'},
										autonomy={'finished': Autonomy.Inherit})

			# x:570 y:195
			OperatableStateMachine.add('ApproachVictim',
										_sm_approachvictim_4,
										transitions={'finished': 'done'},
										autonomy={'finished': Autonomy.Inherit})

			# x:390 y:539
			OperatableStateMachine.add('Cancel',
										_sm_cancel_3,
										transitions={'done': 'done'},
										autonomy={'done': Autonomy.Inherit})

			# x:485 y:380
			OperatableStateMachine.add('LookAt',
										_sm_lookat_2,
										transitions={'done': 'done'},
										autonomy={'done': Autonomy.Inherit})

			# x:432 y:459
			OperatableStateMachine.add('StartHack',
										_sm_starthack_1,
										transitions={'done': 'done'},
										autonomy={'done': Autonomy.Inherit})

			# x:520 y:102
			OperatableStateMachine.add('ExploreVictim',
										_sm_explorevictim_0,
										transitions={'done': 'done'},
										autonomy={'done': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
