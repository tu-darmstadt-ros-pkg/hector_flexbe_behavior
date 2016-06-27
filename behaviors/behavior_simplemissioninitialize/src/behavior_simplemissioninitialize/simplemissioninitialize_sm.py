#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_simplemissioninitialize')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hector_flexbe_states.StartCheck import StartCheck
from hector_flexbe_states.Mapping import Mapping
from hector_flexbe_states.mark_point import MarkPoint
from flexbe_states.operator_decision_state import OperatorDecisionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import PoseStamped
# [/MANUAL_IMPORT]


'''
Created on Thu Jun 02 2016
@author: Gabriel
'''
class SimpleMissionInitializeSM(Behavior):
	'''
	All steps which need to be done during startup
	'''


	def __init__(self):
		super(SimpleMissionInitializeSM, self).__init__()
		self.name = 'SimpleMissionInitialize'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished'], output_keys=['startPoint', 'endPoint'])
		_state_machine.userdata.startPoint = PoseStamped()
		_state_machine.userdata.switchTrue = True
		_state_machine.userdata.endPoint = PoseStamped()
		_state_machine.userdata.switchFalse = False

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:95
			OperatableStateMachine.add('StartCheck',
										StartCheck(),
										transitions={'succeeded': 'ActivateMapping'},
										autonomy={'succeeded': Autonomy.Off})

			# x:198 y:97
			OperatableStateMachine.add('ActivateMapping',
										Mapping(),
										transitions={'succeeded': 'Startpoint'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'switch': 'switchTrue'})

			# x:404 y:99
			OperatableStateMachine.add('Startpoint',
										MarkPoint(),
										transitions={'succeeded': 'Operator_Drive'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'pose': 'startPoint'})

			# x:385 y:263
			OperatableStateMachine.add('Mark_Endpoint',
										MarkPoint(),
										transitions={'succeeded': 'Deactivate_Mapping'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'pose': 'endPoint'})

			# x:346 y:358
			OperatableStateMachine.add('Deactivate_Mapping',
										Mapping(),
										transitions={'succeeded': 'finished'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'switch': 'switchFalse'})

			# x:384 y:176
			OperatableStateMachine.add('Operator_Drive',
										OperatorDecisionState(outcomes=['done'], hint="Drive robot to end pose", suggestion=None),
										transitions={'done': 'Mark_Endpoint'},
										autonomy={'done': Autonomy.Full})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
