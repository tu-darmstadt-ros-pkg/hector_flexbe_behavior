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
from hector_flexbe_states.MarkPoint import MarkPoint
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
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'], output_keys=['startPoint'])
		_state_machine.userdata.startPoint = PoseStamped()

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
										autonomy={'succeeded': Autonomy.Off})

			# x:404 y:99
			OperatableStateMachine.add('Startpoint',
										MarkPoint(),
										transitions={'succeeded': 'finished'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'pose': 'startPoint'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
