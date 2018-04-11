#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hector_flexbe_states.StartCheck import StartCheck
from flexbe_states.operator_decision_state import OperatorDecisionState
from hector_flexbe_states.get_robot_pose import GetRobotPose
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
		_state_machine.userdata.endPoint = PoseStamped()

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:95
			OperatableStateMachine.add('StartCheck',
										StartCheck(),
										transitions={'succeeded': 'Get_Start_Point'},
										autonomy={'succeeded': Autonomy.Off})

			# x:384 y:176
			OperatableStateMachine.add('Operator_Drive',
										OperatorDecisionState(outcomes=['done'], hint="Drive robot to end pose", suggestion=None),
										transitions={'done': 'Get_End_Point'},
										autonomy={'done': Autonomy.Full})

			# x:386 y:74
			OperatableStateMachine.add('Get_Start_Point',
										GetRobotPose(),
										transitions={'succeeded': 'Operator_Drive'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'pose': 'startPoint'})

			# x:394 y:260
			OperatableStateMachine.add('Get_End_Point',
										GetRobotPose(),
										transitions={'succeeded': 'finished'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'pose': 'endPoint'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
