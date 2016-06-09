#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_simplemissiondriveto')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from hector_flexbe_states.drive_to_new import Drive_to_new
from hector_flexbe_states.Wait_DriveTo_new import Wait_DriveTo_new
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jun 07 2016
@author: Gabriel Elisa
'''
class SimpleMissionDriveToSM(Behavior):
	'''
	Drive to exact given point
	'''


	def __init__(self):
		super(SimpleMissionDriveToSM, self).__init__()
		self.name = 'SimpleMissionDriveTo'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:365, x:130 y:365
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.pose = PoseStamped()
		_state_machine.userdata.goalId = 'abc'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:30 y:40
			OperatableStateMachine.add('Drive',
										Drive_to_new(),
										transitions={'succeeded': 'Wait'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'pose': 'pose'})

			# x:191 y:77
			OperatableStateMachine.add('Wait',
										Wait_DriveTo_new(useMoveBase=True),
										transitions={'succeeded': 'finished', 'aborted': 'failed', 'waiting': 'Wait'},
										autonomy={'succeeded': Autonomy.Off, 'aborted': Autonomy.Off, 'waiting': Autonomy.Off},
										remapping={'goalId': 'goalId'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
