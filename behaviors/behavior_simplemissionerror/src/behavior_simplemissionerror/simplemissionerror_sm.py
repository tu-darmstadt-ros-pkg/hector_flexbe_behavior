#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_simplemissionerror')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hector_flexbe_states.ErrorOperator import ErrorOperator
from hector_flexbe_states.mark_point import MarkPoint
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import PoseStamped
# [/MANUAL_IMPORT]


'''
Created on Thu Jun 09 2016
@author: Gabriel Elisa
'''
class SimpleMissionErrorSM(Behavior):
	'''
	Error handling for Simple Mission
	'''


	def __init__(self):
		super(SimpleMissionErrorSM, self).__init__()
		self.name = 'SimpleMissionError'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:130 y:365, x:230 y:365, x:325 y:378
		_state_machine = OperatableStateMachine(outcomes=['failed', 'toStart', 'toEnd'], input_keys=['startPoint', 'endPoint'], output_keys=['startPoint', 'endPoint'])
		_state_machine.userdata.startPoint = PoseStamped()
		_state_machine.userdata.endPoint = PoseStamped()

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:209 y:34
			OperatableStateMachine.add('Operator',
										ErrorOperator(),
										transitions={'failed': 'failed', 'toStart': 'toStart', 'toEnd': 'toEnd', 'setSart': 'setStartpoint', 'setEnd': 'setEndpoint'},
										autonomy={'failed': Autonomy.High, 'toStart': Autonomy.High, 'toEnd': Autonomy.High, 'setSart': Autonomy.High, 'setEnd': Autonomy.High},
										remapping={'startPoint': 'startPoint', 'endPoint': 'endPoint'})

			# x:555 y:29
			OperatableStateMachine.add('setStartpoint',
										MarkPoint(),
										transitions={'succeeded': 'Operator'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'pose': 'startPoint'})

			# x:553 y:120
			OperatableStateMachine.add('setEndpoint',
										MarkPoint(),
										transitions={'succeeded': 'Operator'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'pose': 'endPoint'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
