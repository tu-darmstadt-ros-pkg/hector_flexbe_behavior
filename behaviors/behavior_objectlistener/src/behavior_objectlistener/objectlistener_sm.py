#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_objectlistener')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hector_flexbe_states.object_detection import Object_Detection
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue May 31 2016
@author: Gabriel
'''
class ObjectListenerSM(Behavior):
	'''
	Waits for Object to become active
	'''


	def __init__(self):
		super(ObjectListenerSM, self).__init__()
		self.name = 'ObjectListener'

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
			# x:175 y:158
			OperatableStateMachine.add('Detect',
										Object_Detection(),
										transitions={'continue': 'Detect', 'found': 'finished'},
										autonomy={'continue': Autonomy.Off, 'found': Autonomy.Off},
										remapping={'pose': 'pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
