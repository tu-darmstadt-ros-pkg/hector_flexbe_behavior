#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_concurrency_test')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from behavior_behavior_exploration.behavior_exploration_sm import behavior_explorationSM
from behavior_getclosertovictim.getclosertovictim_sm import GetCloserToVictimSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed May 04 2016
@author: Elisa und Gabriel
'''
class Concurrency_testSM(Behavior):
	'''
	Exploration und get_closer parallel
	'''


	def __init__(self):
		super(Concurrency_testSM, self).__init__()
		self.name = 'Concurrency_test'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(behavior_explorationSM, 'Container/behavior_exploration')
		self.add_behavior(GetCloserToVictimSM, 'Container/GetCloserToVictim')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:322, x:130 y:322
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:322, x:130 y:322, x:230 y:322
		_sm_container_0 = ConcurrencyContainer(outcomes=['finished', 'failed'], conditions=[
										('finished', [('behavior_exploration', 'finished'), ('GetCloserToVictim', 'done')])
										])

		with _sm_container_0:
			# x:30 y:40
			OperatableStateMachine.add('behavior_exploration',
										self.use_behavior(behavior_explorationSM, 'Container/behavior_exploration'),
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Inherit})

			# x:30 y:122
			OperatableStateMachine.add('GetCloserToVictim',
										self.use_behavior(GetCloserToVictimSM, 'Container/GetCloserToVictim'),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Inherit})



		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Container',
										_sm_container_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
