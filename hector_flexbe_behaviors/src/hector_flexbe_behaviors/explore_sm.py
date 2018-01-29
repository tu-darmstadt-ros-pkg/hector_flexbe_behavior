#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_explore')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hector_flexbe_states.LookAtPattern import LookAtPattern
from hector_flexbe_states.error_exploration import ErrorExploration
from hector_flexbe_states.explore import Explore
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Sat May 07 2016
@author: Gabriel und Elisa
'''
class ExploreSM(Behavior):
	'''
	Explores and creates a map of it's surroundings
	'''


	def __init__(self):
		super(ExploreSM, self).__init__()
		self.name = 'Explore'

		# parameters of this behavior
		self.add_parameter('speed', 0.1)

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:48 y:364
		_state_machine = OperatableStateMachine(outcomes=['finished'])
		_state_machine.userdata.lookAround = 'look_around'
		_state_machine.userdata.speed = self.speed

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:33 y:51
			OperatableStateMachine.add('Look_Around',
										LookAtPattern(),
										transitions={'succeeded': 'Explore', 'failed': 'Error'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'pattern': 'lookAround'})

			# x:252 y:133
			OperatableStateMachine.add('Error',
										ErrorExploration(),
										transitions={'restart': 'Look_Around'},
										autonomy={'restart': Autonomy.Off})

			# x:471 y:49
			OperatableStateMachine.add('Explore',
										Explore(),
										transitions={'succeeded': 'Error', 'failed': 'Error'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'speed': 'speed'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
