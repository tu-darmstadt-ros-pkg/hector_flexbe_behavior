#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_simplemissionman4')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from behavior_simplemissioninitialize.simplemissioninitialize_sm import SimpleMissionInitializeSM
from behavior_simplemissiondriveto.simplemissiondriveto_sm import SimpleMissionDriveToSM
from flexbe_states.operator_decision_state import OperatorDecisionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import PoseStamped
# [/MANUAL_IMPORT]


'''
Created on Thu Jun 02 2016
@author: Gabriel
'''
class SimpleMissionMan4SM(Behavior):
	'''
	Simple mission structure
	'''


	def __init__(self):
		super(SimpleMissionMan4SM, self).__init__()
		self.name = 'SimpleMissionMan4'

		# parameters of this behavior
		self.add_parameter('speed', 0.1)
		self.add_parameter('allow_backwards', False)

		# references to used behaviors
		self.add_behavior(SimpleMissionInitializeSM, 'SimpleMissionInitialize')
		self.add_behavior(SimpleMissionDriveToSM, 'DriveToEnd')
		self.add_behavior(SimpleMissionDriveToSM, 'DriveToStart')

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:52 y:481, x:134 y:482
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.pose = PoseStamped()
		_state_machine.userdata.startPoint = PoseStamped()
		_state_machine.userdata.endPoint = PoseStamped()
		_state_machine.userdata.switchFalse = False

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:31 y:51
			OperatableStateMachine.add('SimpleMissionInitialize',
										self.use_behavior(SimpleMissionInitializeSM, 'SimpleMissionInitialize'),
										transitions={'finished': 'OperatorDrive'},
										autonomy={'finished': Autonomy.Inherit},
										remapping={'startPoint': 'startPoint', 'endPoint': 'endPoint'})

			# x:656 y:237
			OperatableStateMachine.add('DriveToEnd',
										self.use_behavior(SimpleMissionDriveToSM, 'DriveToEnd'),
										transitions={'finished': 'DriveToStart', 'failed': 'DriveToEnd'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pose': 'endPoint'})

			# x:243 y:241
			OperatableStateMachine.add('DriveToStart',
										self.use_behavior(SimpleMissionDriveToSM, 'DriveToStart'),
										transitions={'finished': 'DriveToEnd', 'failed': 'DriveToStart'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'pose': 'startPoint'})

			# x:477 y:84
			OperatableStateMachine.add('OperatorDrive',
										OperatorDecisionState(outcomes=['toStart', 'toEnd'], hint='Operator Drive', suggestion=None),
										transitions={'toStart': 'DriveToStart', 'toEnd': 'DriveToEnd'},
										autonomy={'toStart': Autonomy.Off, 'toEnd': Autonomy.Off})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
