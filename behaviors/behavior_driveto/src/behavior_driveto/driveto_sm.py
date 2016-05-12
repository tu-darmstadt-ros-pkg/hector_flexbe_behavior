#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_driveto')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hector_flexbe_states.Send_Request_new import Send_Request_new
from hector_flexbe_states.Wait_DriveTo_new import Wait_DriveTo_new
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from hector_move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import PoseStamped
# [/MANUAL_IMPORT]


'''
Created on Tue Dec 29 2015
@author: Elisa, Gabriel
'''
class DriveToSM(Behavior):
	'''
	statemachine drive_to
	'''


	def __init__(self):
		super(DriveToSM, self).__init__()
		self.name = 'DriveTo'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:322, x:130 y:322
		_state_machine = OperatableStateMachine(outcomes=['succeeded', 'aborted'], input_keys=['pose', 'params_distance'])
		_state_machine.userdata.goalId = ''
		_state_machine.userdata.params_distance = 0
		_state_machine.userdata.pose = PoseStamped()

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]

		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('send_request',
										Send_Request_new(useMoveBase=True),
										transitions={'succeeded': 'wait'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'goalId': 'goalId', 'params_distance': 'params_distance', 'pose': 'pose'})

			# x:223 y:96
			OperatableStateMachine.add('wait',
										Wait_DriveTo_new(useMoveBase=True),
										transitions={'succeeded': 'succeeded', 'aborted': 'aborted', 'waiting': 'wait'},
										autonomy={'succeeded': Autonomy.Off, 'aborted': Autonomy.Off, 'waiting': Autonomy.Off},
										remapping={'goalId': 'goalId'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
