#!/usr/bin/env python
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

import roslib; roslib.load_manifest('behavior_getclosertovictim')
from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, Logger
from hector_flexbe_states.confirm_victim import confirm_victim
from hector_flexbe_states.Wait_getCloserVictim import Wait_getCloserVictim
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Jan 07 2016
@author: Elisa, Gabriel
'''
class GetCloserToVictimSM(Behavior):
	'''
	statemachine GetCloserToVictim
	'''


	def __init__(self):
		super(GetCloserToVictimSM, self).__init__()
		self.name = 'GetCloserToVictim'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:322, x:130 y:322
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.pose_position_y = 0
		_state_machine.userdata.pose_position_x = 0
		_state_machine.userdata.pose_orientation_y = 0
		_state_machine.userdata.pose_orientation_z = 0
		_state_machine.userdata.pose_orientation_w = 0
		_state_machine.userdata.pose_orientation_x = 0
		_state_machine.userdata.task_details_task_id = 0
		_state_machine.userdata.params_distance = 0
		_state_machine.userdata.pose_position_z = 0

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:54 y:157
			OperatableStateMachine.add('confirm_victim',
										confirm_victim(),
										transitions={'succeeded': 'wait'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'task_details_task_id': 'task_details_task_id'})

			# x:337 y:152
			OperatableStateMachine.add('wait',
										Wait_getCloserVictim(),
										transitions={'waiting': 'wait', 'restart': 'wait', 'preempted': 'finished'},
										autonomy={'waiting': Autonomy.Off, 'restart': Autonomy.Off, 'preempted': Autonomy.Off},
										remapping={'task_details_task_id': 'task_details_task_id', 'params_distance': 'params_distance', 'pose_position_x': 'pose_position_x', 'pose_position_y': 'pose_position_y', 'pose_position_z': 'pose_position_z', 'pose_orientation_x': 'pose_orientation_x', 'pose_orientation_y': 'pose_orientation_y', 'pose_orientation_z': 'pose_orientation_z', 'pose_orientation_w': 'pose_orientation_w'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
