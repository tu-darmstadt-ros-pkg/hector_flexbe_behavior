#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_set_points')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hector_flexbe_states.get_robot_pose import GetRobotPose
from flexbe_states.operator_decision_state import OperatorDecisionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from geometry_msgs.msg import PoseStamped
# [/MANUAL_IMPORT]


'''
Created on Thu Jun 02 2016
@author: Gabriel
'''
class SetPointsSM(Behavior):
	'''
	All steps which need to be done during startup
	'''


	def __init__(self):
		super(SetPointsSM, self).__init__()
		self.name = 'Set Points'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:59 y:401
		_state_machine = OperatableStateMachine(outcomes=['finished'], output_keys=['startPoint', 'endPoint'])
		_state_machine.userdata.startPoint = PoseStamped()
		_state_machine.userdata.endPoint = PoseStamped()

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:34 y:51
			OperatableStateMachine.add('Get_Start_Point',
										GetRobotPose(),
										transitions={'succeeded': 'Operator_Drive'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'pose': 'startPoint'})

			# x:35 y:294
			OperatableStateMachine.add('Get_End_Point',
										GetRobotPose(),
										transitions={'succeeded': 'finished'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'pose': 'endPoint'})

			# x:27 y:178
			OperatableStateMachine.add('Operator_Drive',
										OperatorDecisionState(outcomes=['done'], hint="Drive robot to end pose", suggestion=None),
										transitions={'done': 'Get_End_Point'},
										autonomy={'done': Autonomy.Full})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
