#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexbe_states.wait_state import WaitState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jan 15 2019
@author: Gabriel Huettenberger
'''
class GenericStaticTestMethodSM(Behavior):
	'''
	Behavior for executing a generic test method on a known map
	'''


	def __init__(self):
		super(GenericStaticTestMethodSM, self).__init__()
		self.name = 'Generic Static Test Method'

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

		# x:305 y:367, x:154 y:364, x:230 y:365, x:72 y:363, x:430 y:365
		_sm_find_end_point_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('failed', [('explore', 'done')]),
										('finished', [('find_end_point', 'done')]),
										('failed', [('timeout', 'done')])
										])

		with _sm_find_end_point_0:
			# x:53 y:43
			OperatableStateMachine.add('explore',
										WaitState(wait_time=1),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:179 y:38
			OperatableStateMachine.add('find_end_point',
										WaitState(wait_time=1),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:342 y:41
			OperatableStateMachine.add('timeout',
										WaitState(wait_time=60),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})


		# x:564 y:329
		_sm_find_start_end_point_1 = OperatableStateMachine(outcomes=['finished'])

		with _sm_find_start_end_point_1:
			# x:30 y:40
			OperatableStateMachine.add('set_start_point',
										WaitState(wait_time=1),
										transitions={'done': 'find_end_point'},
										autonomy={'done': Autonomy.Off})

			# x:183 y:57
			OperatableStateMachine.add('find_end_point',
										_sm_find_end_point_0,
										transitions={'finished': 'drive_to_end_point', 'failed': 'manually_set_end_point'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:371 y:54
			OperatableStateMachine.add('drive_to_end_point',
										WaitState(wait_time=1),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:173 y:222
			OperatableStateMachine.add('manually_set_end_point',
										WaitState(wait_time=1),
										transitions={'done': 'drive_to_end_point'},
										autonomy={'done': Autonomy.Off})


		# x:30 y:365, x:130 y:365, x:230 y:365, x:330 y:365, x:430 y:365, x:530 y:365
		_sm_map_exploration_2 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('find_start_end_point', 'finished')]),
										('failed', [('find_inspection_points', 'done')]),
										('failed', [('find_manipulation_points', 'done')])
										])

		with _sm_map_exploration_2:
			# x:30 y:40
			OperatableStateMachine.add('find_start_end_point',
										_sm_find_start_end_point_1,
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Inherit})

			# x:211 y:38
			OperatableStateMachine.add('find_inspection_points',
										WaitState(wait_time=1),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})

			# x:398 y:31
			OperatableStateMachine.add('find_manipulation_points',
										WaitState(wait_time=1),
										transitions={'done': 'failed'},
										autonomy={'done': Autonomy.Off})


		# x:30 y:365, x:130 y:365
		_sm_report_results_3 = OperatableStateMachine(outcomes=['finished', 'failed'])

		with _sm_report_results_3:
			# x:51 y:51
			OperatableStateMachine.add('export_mission_results',
										WaitState(wait_time=1),
										transitions={'done': 'export_map'},
										autonomy={'done': Autonomy.Off})

			# x:372 y:186
			OperatableStateMachine.add('export_state_transition_log',
										WaitState(wait_time=1),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:252 y:49
			OperatableStateMachine.add('export_map',
										WaitState(wait_time=1),
										transitions={'done': 'export_state_transition_log'},
										autonomy={'done': Autonomy.Off})



		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('PLACEHOLDER',
										WaitState(wait_time=1),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off})

			# x:596 y:55
			OperatableStateMachine.add('load_mission',
										WaitState(wait_time=1),
										transitions={'done': 'check_missing_points'},
										autonomy={'done': Autonomy.Off})

			# x:603 y:476
			OperatableStateMachine.add('execute_mission',
										WaitState(wait_time=1),
										transitions={'done': 'report_results'},
										autonomy={'done': Autonomy.Off})

			# x:289 y:474
			OperatableStateMachine.add('report_results',
										_sm_report_results_3,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:243 y:53
			OperatableStateMachine.add('Map exploration',
										_sm_map_exploration_2,
										transitions={'finished': 'load_mission', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})

			# x:579 y:241
			OperatableStateMachine.add('check_missing_points',
										WaitState(wait_time=1),
										transitions={'done': 'execute_mission'},
										autonomy={'done': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
