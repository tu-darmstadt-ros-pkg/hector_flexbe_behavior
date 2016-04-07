#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_hectorbehavior1')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, Logger
from hector_flexbe_states.behavior_init import behavior_init
from hector_flexbe_states.START_Exploration_Transform import START_Exploration_Transform
from hector_flexbe_states.Error_Exploration import Error_Exploration
from hector_flexbe_states.Wait_Exploration import Wait_Exploration
from hector_flexbe_states.behavior2 import behavior2
from hector_flexbe_states.Send_Request2 import Send_Request2
from hector_flexbe_states.Wait_DriveTo import Wait_DriveTo
from hector_flexbe_states.Wait_getCloserVictim import Wait_getCloserVictim
from hector_flexbe_states.confirm_victim import confirm_victim
from hector_flexbe_states.verify_victim import verify_victim
from hector_flexbe_states.start_selected_direction import start_selected_direction
from hector_flexbe_states.Wait_StartHack import Wait_StartHack
from hector_flexbe_states.prepare_turn import prepare_turn
from hector_flexbe_states.execute_turn import execute_turn
from hector_flexbe_states.complete_task_StartHack import complete_task_StartHack
from hector_flexbe_states.complete_task_LookAt import complete_task_LookAt
from hector_flexbe_states.Cancel import Cancel
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Dec 29 2015
@author: Elisa, Gabriel
'''
class HectorBehavior1SM(Behavior):
	'''
	first try
	'''


	def __init__(self):
		super(HectorBehavior1SM, self).__init__()
		self.name = 'HectorBehavior1'

		# parameters of this behavior
		self.add_parameter('config', dict())

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:941 y:348, x:257 y:16
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.goalId = 'none'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:322, x:130 y:322
		_sm_drive_to_0 = OperatableStateMachine(outcomes=['succeeded', 'aborted'], input_keys=['goalId'])

		with _sm_drive_to_0:
			# x:59 y:35
			OperatableStateMachine.add('send_request',
										Send_Request2(),
										transitions={'succeeded': 'wait'},
										autonomy={'succeeded': Autonomy.Off})

			# x:66 y:171
			OperatableStateMachine.add('wait',
										Wait_DriveTo(useMoveBase=True),
										transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 'waiting': 'wait'},
										autonomy={'succeeded': Autonomy.Off, 'aborted': Autonomy.Off, 'waiting': Autonomy.Off},
										remapping={'goalId': 'goalId'})


		# x:30 y:322, x:130 y:322
		_sm_drive_to_victim_1 = OperatableStateMachine(outcomes=['succeeded', 'aborted'], input_keys=['goalId'])

		with _sm_drive_to_victim_1:
			# x:161 y:64
			OperatableStateMachine.add('send_request',
										Send_Request2(),
										transitions={'succeeded': 'wait'},
										autonomy={'succeeded': Autonomy.Off})

			# x:129 y:192
			OperatableStateMachine.add('wait',
										Wait_DriveTo(useMoveBase=True),
										transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 'waiting': 'wait'},
										autonomy={'succeeded': Autonomy.Off, 'aborted': Autonomy.Off, 'waiting': Autonomy.Off},
										remapping={'goalId': 'goalId'})


		# x:30 y:322, x:130 y:322
		_sm_approach_victim_2 = OperatableStateMachine(outcomes=['succeeded', 'aborted'], input_keys=['goalId'])

		with _sm_approach_victim_2:
			# x:105 y:84
			OperatableStateMachine.add('send_request',
										Send_Request2(),
										transitions={'succeeded': 'wait'},
										autonomy={'succeeded': Autonomy.Off})

			# x:115 y:299
			OperatableStateMachine.add('wait',
										Wait_DriveTo(useMoveBase=True),
										transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 'waiting': 'wait'},
										autonomy={'succeeded': Autonomy.Off, 'aborted': Autonomy.Off, 'waiting': Autonomy.Off},
										remapping={'goalId': 'goalId'})


		# x:30 y:322
		_sm_cancel_3 = OperatableStateMachine(outcomes=['done'])

		with _sm_cancel_3:
			# x:86 y:176
			OperatableStateMachine.add('cancel',
										Cancel(),
										transitions={'done': 'done'},
										autonomy={'done': Autonomy.Off})


		# x:30 y:322
		_sm_lookat_4 = OperatableStateMachine(outcomes=['done'], input_keys=['goalId'])

		with _sm_lookat_4:
			# x:131 y:61
			OperatableStateMachine.add('Drive_To',
										_sm_drive_to_0,
										transitions={'succeeded': 'complete_task', 'aborted': 'complete_task'},
										autonomy={'succeeded': Autonomy.Inherit, 'aborted': Autonomy.Inherit},
										remapping={'goalId': 'goalId'})

			# x:128 y:237
			OperatableStateMachine.add('complete_task',
										complete_task_LookAt(),
										transitions={'error': 'complete_task', 'done': 'done'},
										autonomy={'error': Autonomy.Off, 'done': Autonomy.Off})


		# x:30 y:322
		_sm_starthack_5 = OperatableStateMachine(outcomes=['done'])

		with _sm_starthack_5:
			# x:184 y:83
			OperatableStateMachine.add('start_selected_direction',
										start_selected_direction(),
										transitions={'succeeded': 'wait', 'skip': 'complete_task', 'turn': 'prepare_turn'},
										autonomy={'succeeded': Autonomy.Off, 'skip': Autonomy.Off, 'turn': Autonomy.Off})

			# x:41 y:147
			OperatableStateMachine.add('wait',
										Wait_StartHack(),
										transitions={'true': 'wait', 'false': 'complete_task'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off})

			# x:472 y:166
			OperatableStateMachine.add('prepare_turn',
										prepare_turn(),
										transitions={'done': 'complete_task', 'turn': 'execute_turn'},
										autonomy={'done': Autonomy.Off, 'turn': Autonomy.Off})

			# x:482 y:404
			OperatableStateMachine.add('execute_turn',
										execute_turn(),
										transitions={'succeeded': 'prepare_turn', 'aborted': 'prepare_turn', 'done': 'done'},
										autonomy={'succeeded': Autonomy.Off, 'aborted': Autonomy.Off, 'done': Autonomy.Off})

			# x:187 y:268
			OperatableStateMachine.add('complete_task',
										complete_task_StartHack(),
										transitions={'error': 'complete_task', 'done': 'done'},
										autonomy={'error': Autonomy.Off, 'done': Autonomy.Off})


		# x:30 y:322
		_sm_explorevictim_6 = OperatableStateMachine(outcomes=['done'], input_keys=['goalId'])

		with _sm_explorevictim_6:
			# x:142 y:55
			OperatableStateMachine.add('Drive_to_Victim',
										_sm_drive_to_victim_1,
										transitions={'succeeded': 'verify_victim', 'aborted': 'verify_victim'},
										autonomy={'succeeded': Autonomy.Inherit, 'aborted': Autonomy.Inherit},
										remapping={'goalId': 'goalId'})

			# x:126 y:337
			OperatableStateMachine.add('confirm_victim',
										confirm_victim(),
										transitions={'succeeded': 'done'},
										autonomy={'succeeded': Autonomy.Off})

			# x:138 y:234
			OperatableStateMachine.add('verify_victim',
										verify_victim(),
										transitions={'true': 'confirm_victim'},
										autonomy={'true': Autonomy.Off})


		# x:30 y:322
		_sm_getclosertovictim_7 = OperatableStateMachine(outcomes=['finished'], input_keys=['goalId'])

		with _sm_getclosertovictim_7:
			# x:152 y:101
			OperatableStateMachine.add('Approach_victim',
										_sm_approach_victim_2,
										transitions={'succeeded': 'confirm_victim', 'aborted': 'confirm_victim'},
										autonomy={'succeeded': Autonomy.Inherit, 'aborted': Autonomy.Inherit},
										remapping={'goalId': 'goalId'})

			# x:547 y:243
			OperatableStateMachine.add('Wait',
										Wait_getCloserVictim(),
										transitions={'waiting': 'Wait', 'restart': 'Approach_victim'},
										autonomy={'waiting': Autonomy.Off, 'restart': Autonomy.Off})

			# x:74 y:331
			OperatableStateMachine.add('confirm_victim',
										confirm_victim(),
										transitions={'succeeded': 'Wait'},
										autonomy={'succeeded': Autonomy.Off})


		# x:30 y:322
		_sm_exploration_8 = OperatableStateMachine(outcomes=['finished'], input_keys=['goalId'])

		with _sm_exploration_8:
			# x:107 y:46
			OperatableStateMachine.add('Start_Exploration',
										START_Exploration_Transform(),
										transitions={'succeeded': 'Wait'},
										autonomy={'succeeded': Autonomy.Off})

			# x:442 y:239
			OperatableStateMachine.add('Error',
										Error_Exploration(),
										transitions={'restart': 'Start_Exploration'},
										autonomy={'restart': Autonomy.Off})

			# x:38 y:209
			OperatableStateMachine.add('Wait',
										Wait_Exploration(),
										transitions={'succeeded': 'Error', 'aborted': 'Error', 'waiting': 'Wait'},
										autonomy={'succeeded': Autonomy.Off, 'aborted': Autonomy.Off, 'waiting': Autonomy.Off},
										remapping={'goalId': 'goalId'})


		# x:757 y:284, x:130 y:322
		_sm_container_9 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['goalId'])

		with _sm_container_9:
			# x:59 y:78
			OperatableStateMachine.add('behavior',
										behavior_init(),
										transitions={'continue': 'behavior2', 'failed': 'failed'},
										autonomy={'continue': Autonomy.Off, 'failed': Autonomy.Off})

			# x:444 y:122
			OperatableStateMachine.add('Exploration',
										_sm_exploration_8,
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'goalId': 'goalId'})

			# x:64 y:228
			OperatableStateMachine.add('behavior2',
										behavior2(),
										transitions={'APPROACH_VICTIM': 'GetCloserToVictim', 'EXPLORE': 'Exploration', 'EXPLORE_VICTIM_HYPOTHESIS': 'ExploreVictim', 'GO_TO': 'behavior2', 'START': 'StartHack', 'LOOK_AT': 'LookAt', 'CANCEL': 'Cancel'},
										autonomy={'APPROACH_VICTIM': Autonomy.Off, 'EXPLORE': Autonomy.Off, 'EXPLORE_VICTIM_HYPOTHESIS': Autonomy.Off, 'GO_TO': Autonomy.Off, 'START': Autonomy.Off, 'LOOK_AT': Autonomy.Off, 'CANCEL': Autonomy.Off})

			# x:430 y:22
			OperatableStateMachine.add('GetCloserToVictim',
										_sm_getclosertovictim_7,
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'goalId': 'goalId'})

			# x:442 y:222
			OperatableStateMachine.add('ExploreVictim',
										_sm_explorevictim_6,
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Inherit},
										remapping={'goalId': 'goalId'})

			# x:444 y:322
			OperatableStateMachine.add('StartHack',
										_sm_starthack_5,
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Inherit})

			# x:444 y:422
			OperatableStateMachine.add('LookAt',
										_sm_lookat_4,
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Inherit},
										remapping={'goalId': 'goalId'})

			# x:444 y:522
			OperatableStateMachine.add('Cancel',
										_sm_cancel_3,
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Inherit})


		# x:30 y:322, x:130 y:322, x:230 y:322, x:330 y:322
		_sm_container_10 = ConcurrencyContainer(outcomes=['finished', 'failed'], input_keys=['goalId'], conditions=[
										('finished', [('Container', 'finished')]),
										('failed', [('Container', 'failed')])
										])

		with _sm_container_10:
			# x:30 y:40
			OperatableStateMachine.add('Container',
										_sm_container_9,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'goalId': 'goalId'})



		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Container',
										_sm_container_10,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'goalId': 'goalId'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
