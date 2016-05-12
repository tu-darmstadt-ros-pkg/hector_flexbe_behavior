#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_victimapproach')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from behavior_behavior_exploration.behavior_exploration_sm import behavior_explorationSM
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed May 04 2016
@author: Elisa und Gabriel
'''
class VictimApproachSM(Behavior):
	'''
	Exploration und GetCloser verbinden
	'''


	def __init__(self):
		super(VictimApproachSM, self).__init__()
		self.name = 'VictimApproach'

		# parameters of this behavior

		# references to used behaviors
		self.add_behavior(behavior_explorationSM, 'behavior_exploration')

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


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('behavior_exploration',
										self.use_behavior(behavior_explorationSM, 'behavior_exploration'),
										transitions={'finished': 'finished'},
										autonomy={'finished': Autonomy.Inherit})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
